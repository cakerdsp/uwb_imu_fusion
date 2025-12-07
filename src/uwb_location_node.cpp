#include "uwb_imu_fusion/uwb_location_node.hpp"
#include "uwb_imu_fusion/initializer.hpp" // 引入初始化工具

namespace uwb_imu_fusion {

UwbLocationNode::UwbLocationNode(const rclcpp::NodeOptions& options)
    : Node("Uwb_Location", rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true)) {
    
    RCLCPP_INFO(this->get_logger(), "Starting Uwb_Location Node (Industrial FSM Version).");

    // 1. 加载参数
    load_parameters();

    // 2. 初始化算法 (先给个 Dummy 占位，真正初始化在 STATIC_INIT 结束时)
    if (algo_type_ == "ESKF") {
        fusion_algo_ = std::make_unique<ESKF>();
    } else {
        fusion_algo_ = std::make_unique<DummyAlgo>();
    }

    // 3. 初始化硬件
    init_hardware(); 
    // 如果硬件初始化成功，状态会转为 IDLE (在 init_hardware 里设置)

    // 4. 初始化 ROS 通信
    auto qos = rclcpp::SensorDataQoS();
    std::string imu_topic = this->get_parameter("topics.imu_sub").as_string();
    std::string odom_topic = this->get_parameter("topics.odom_pub").as_string();

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, qos, 
        std::bind(&UwbLocationNode::imu_callback, this, std::placeholders::_1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // 5. 启动主循环定时器 (50Hz)
    int timeout_ms = this->get_parameter("serial.timeout_ms").as_int();
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(timeout_ms),
        std::bind(&UwbLocationNode::timer_callback, this));
}

void UwbLocationNode::load_parameters() {
    // 声明并获取静态参数
    this->declare_parameter("topics.imu_sub", "/imu/data");
    this->declare_parameter("topics.odom_pub", "/odometry/filtered");
    this->declare_parameter("frames.world_frame_id", "map");
    this->declare_parameter("frames.body_frame_id", "base_link");
    this->declare_parameter("serial.port_name", "/dev/ttyUSB0");
    this->declare_parameter("serial.baud_rate", 115200);
    this->declare_parameter("serial.timeout_ms", 20);
    this->declare_parameter("algorithm_type", "Dummy");
    this->declare_parameter("NLOS.nlos_q_threshold", 6.0);

    world_frame_id_ = this->get_parameter("frames.world_frame_id").as_string();
    body_frame_id_ = this->get_parameter("frames.body_frame_id").as_string();
    algo_type_ = this->get_parameter("algorithm_type").as_string();
    nlos_q_threshold_ = this->get_parameter("NLOS.nlos_q_threshold").as_int();
    // 动态加载基站配置
    // 前置条件：必须在构造函数中开启 automatically_declare_parameters_from_overrides(true)
    // list_parameters 只能列出 YAML 中已存在的参数
    auto params_result = this->list_parameters({"anchors"}, 10);
    
    for (const auto& name : params_result.names) {
        // 1. 过滤层级：确保参数名包含 "." 且不是根节点
        size_t last_dot_pos = name.rfind('.');
        if (last_dot_pos == std::string::npos) continue;

        // 2. 防御性声明：虽然开启了自动声明，但显式检查更安全
        if (!this->has_parameter(name)) {
            try {
                this->declare_parameter(name, std::vector<double>{});
            } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&) {
                // 忽略已声明错误
            }
        }

        try {
            // 3. 类型检查与获取
            rclcpp::Parameter param = this->get_parameter(name);
            if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
                RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not a double array, skipping.", name.c_str());
                continue;
            }

            std::vector<double> pos = param.as_double_array();
            
            // 4. 数据维度检查
            if (pos.size() != 3) {
                RCLCPP_WARN(this->get_logger(), "Anchor '%s' dimension error. Expected 3 (x,y,z), got %zu", name.c_str(), pos.size());
                continue;
            }

            // 5. 健壮的 ID 解析
            // 提取最后一段名称 (例如 "anchors.room1.A0" -> "A0")
            std::string anchor_key = name.substr(last_dot_pos + 1);
            
            // 校验格式：必须以 'A' 开头，且后续部分必须为数字
            if (anchor_key.empty() || anchor_key[0] != 'A' || anchor_key.size() < 2) {
                RCLCPP_WARN(this->get_logger(), "Invalid anchor name format '%s'. Expected 'A<ID>' (e.g., A0)", anchor_key.c_str());
                continue;
            }

            std::string id_str = anchor_key.substr(1);
            // 检查剩余部分是否全为数字
            if (id_str.find_first_not_of("0123456789") != std::string::npos) {
                RCLCPP_WARN(this->get_logger(), "Invalid anchor ID '%s'. Expected numeric ID.", id_str.c_str());
                continue;
            }

            int id = std::stoi(id_str);

            // 6. 存入 Map
            anchors_[id] = Eigen::Vector3d(pos[0], pos[1], pos[2]);
            
            RCLCPP_INFO(this->get_logger(), "Loaded Anchor %s (ID %d): [%.2f, %.2f, %.2f]", 
                anchor_key.c_str(), id, pos[0], pos[1], pos[2]);

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error loading parameter '%s': %s", name.c_str(), e.what());
        }
    }

    if (anchors_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No anchors loaded! Check config.yaml indentation and naming.");
    }
}

void UwbLocationNode::init_hardware() {
    std::string port = this->get_parameter("serial.port_name").as_string();
    int baud = this->get_parameter("serial.baud_rate").as_int();

    if (serial_reader_.open(port, baud)) {
        RCLCPP_INFO(this->get_logger(), "Serial port opened: %s", port.c_str());
        sys_state_ = SysState::IDLE; // 硬件OK，进入待机
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port!");
        sys_state_ = SysState::SYSTEM_ERROR;
    }
}

// -------------------------------------------------------------------------
// IMU 回调: 高频 (100Hz+)
// -------------------------------------------------------------------------
void UwbLocationNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    ImuMeasurement imu;
    rclcpp::Time stamp = msg->header.stamp;
    imu.timestamp = stamp.seconds();
    imu.acc << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
    imu.gyro << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;

    // 状态机分流
    switch (sys_state_) {
        case SysState::STATIC_INIT:
            // 初始化阶段：只存数据，不跑算法
            init_imu_buf_.push_back(imu);
            break;

        case SysState::RUNNING_FUSION:
        case SysState::RUNNING_COASTING:
            // 运行阶段：喂给算法进行预测 (Predict)
            fusion_algo_->addImuData(imu);
            
            // 【关键】利用高频 IMU 发布 TF，保证丝滑
            // 注意：此时 getCurrentState 返回的是刚刚 Predict 后的状态
            {
                NavState state = fusion_algo_->getCurrentState();
                publish_tf(state, stamp);
            }
            break;

        default:
            break; // 其他状态忽略 IMU
    }
}

// -------------------------------------------------------------------------
// 定时器回调: 低频 (50Hz) - 处理 UWB + 状态流转 + Odom发布
// -------------------------------------------------------------------------
void UwbLocationNode::timer_callback() {
    rclcpp::Time current_ros_time = this->now();
    double current_ts = current_ros_time.seconds();

    // 读取 UWB 数据
    auto frames = serial_reader_.read_and_parse();

    // 状态机主逻辑
    switch (sys_state_) {
        
        // -----------------------------------------------------
        // 阶段 0: 异常
        // -----------------------------------------------------
        case SysState::SYSTEM_ERROR:
            // 可在此处尝试重连串口...
            return;

        // -----------------------------------------------------
        // 阶段 1: IDLE (等待传感器数据就绪)
        // -----------------------------------------------------
        case SysState::IDLE:
            // 检查是否有有效数据流入
            if (!frames.empty()) {
                // 收到第一帧数据，开始静止初始化
                sys_state_ = SysState::STATIC_INIT;
                static_init_start_time_ = current_ros_time;
                
                // 清空 Buffer
                init_imu_buf_.clear();
                init_uwb_buf_.clear();
                
                RCLCPP_INFO(this->get_logger(), "Sensors detected. Start STATIC INIT (2s). Please keep robot STILL!");
            }
            break;

        // -----------------------------------------------------
        // 阶段 2: STATIC_INIT (静止初始化)
        // -----------------------------------------------------
        case SysState::STATIC_INIT: {
            // A. 收集 UWB 数据 (如果有)
            if (!frames.empty()) {
                auto frame = frames.back();
                for (const auto& anchor_data : frame.anchors) {
                    if (anchors_.find(anchor_data.id) == anchors_.end()) continue;
                    UwbMeasurement meas;
                    meas.anchor_pos = anchors_[anchor_data.id];
                    meas.dist = anchor_data.dist;
                    init_uwb_buf_.push_back(meas);
                }
            }
            
            // B. 检查时间是否足够 (2秒)
            if ((current_ros_time - static_init_start_time_).seconds() > 2.0) {
                
                // 检查数据量是否足够
                if (init_imu_buf_.size() < 50) {
                    RCLCPP_WARN(this->get_logger(), "Not enough IMU data for init. Retrying...");
                    static_init_start_time_ = current_ros_time; // 重置计时
                    init_imu_buf_.clear();
                    return;
                }

                // --- 核心初始化逻辑 ---
                RCLCPP_INFO(this->get_logger(), "Calculating initial state...");

                // 1. 算出初始状态 (Roll, Pitch, Bias)
                NavState init_state = Initializer::alignIMU(init_imu_buf_);
                
                // 2. 算出初始位置 (调用你的三边定位库)
                init_state.p = Initializer::solveTrilateration(init_uwb_buf_);
                
                // 3. 补上时间戳
                init_state.timestamp = current_ts;

                // 4. 初始化算法 (ESKF 内部会设置大方差 P 矩阵)
                fusion_algo_->initialize(init_state);
                
                // 5. 切换状态
                sys_state_ = SysState::RUNNING_FUSION;
                last_uwb_time_ = current_ros_time; // 初始化看门狗
                
                RCLCPP_INFO(this->get_logger(), "Initialization Done. ESKF Running (Fusion Mode).");
            }
            break;
        }

        // -----------------------------------------------------
        // 阶段 3: RUNNING_FUSION (正常融合)
        // -----------------------------------------------------
        case SysState::RUNNING_FUSION: {
            // A. 处理 UWB 数据 (Update)
            if (!frames.empty()) {
                auto frame = frames.back(); // 只取最新
                for (const auto& anchor_data : frame.anchors) {
                    if (anchors_.find(anchor_data.id) == anchors_.end()) continue;
                    if (anchor_data.q_value < nlos_q_threshold_) continue; // 质量过滤

                    UwbMeasurement meas;
                    meas.timestamp = current_ts;
                    meas.anchor_id = anchor_data.id;
                    meas.anchor_pos = anchors_[anchor_data.id];
                    meas.dist = anchor_data.dist;
                    meas.q_value = anchor_data.q_value;

                    // 执行更新
                    fusion_algo_->addUwbData(meas);
                }
                // 喂狗
                last_uwb_time_ = current_ros_time;
            } else {
                // 检查超时 (1秒没数据 -> 切盲推)
                if ((current_ros_time - last_uwb_time_).seconds() > 1.0) {
                    sys_state_ = SysState::RUNNING_COASTING;
                    RCLCPP_WARN(this->get_logger(), "UWB Lost! Switching to COASTING mode.");
                }
            }

            // B. 发布 Odometry (TF 已经在 imu_callback 发了)
            NavState state = fusion_algo_->getCurrentState();
            publish_odometry(state, current_ros_time);
            break;
        }

        // -----------------------------------------------------
        // 阶段 4: RUNNING_COASTING (盲推模式)
        // -----------------------------------------------------
        case SysState::RUNNING_COASTING: {
            // A. 检查 UWB 是否恢复
            if (!frames.empty()) {
                // UWB 回来了！
                sys_state_ = SysState::RUNNING_FUSION;
                last_uwb_time_ = current_ros_time;
                RCLCPP_INFO(this->get_logger(), "UWB Recovered. Switching back to FUSION mode.");
                
                // 立即处理这一帧
                // ... (同 FUSION 逻辑，略，下一轮循环会自动处理)
            } else {
                // 检查是否丢太久 (比如 20秒) -> 报错或重置
                if ((current_ros_time - last_uwb_time_).seconds() > 20.0) {
                    RCLCPP_ERROR(this->get_logger(), "UWB Lost for too long (>20s). Navigation unreliable!");
                    // 可选: sys_state_ = SysState::SYSTEM_ERROR;
                }
            }

            // B. 依然发布 Odometry (基于纯 IMU 预测)
            NavState state = fusion_algo_->getCurrentState();
            publish_odometry(state, current_ros_time);
            break;
        }
        
        default:
            break;
    }
}

void UwbLocationNode::publish_odometry(const NavState& state, const rclcpp::Time& stamp) {

    nav_msgs::msg::Odometry msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = world_frame_id_;
    msg.child_frame_id = body_frame_id_;

    // 填充位姿
    msg.pose.pose.position.x = state.p.x();
    msg.pose.pose.position.y = state.p.y();
    msg.pose.pose.position.z = state.p.z();
    msg.pose.pose.orientation.w = state.q.w();
    msg.pose.pose.orientation.x = state.q.x();
    msg.pose.pose.orientation.y = state.q.y();
    msg.pose.pose.orientation.z = state.q.z();

    // 填充速度 （body坐标系下）
    // 之前的代码角速度是传递的IMU的数据，这里我直接置0了
    Eigen::Vector3d v_body = state.q.inverse() * state.v;
    msg.twist.twist.linear.x = v_body.x();
    msg.twist.twist.linear.y = v_body.y();
    msg.twist.twist.linear.z = v_body.z();

    odom_pub_->publish(msg);
}

void UwbLocationNode::publish_tf(const NavState& state, const rclcpp::Time& stamp) {
    geometry_msgs::msg::TransformStamped t;

    // 使用传入的时间戳，或者如果过旧则使用当前时间（视需求而定）
    t.header.stamp = stamp; 
    t.header.frame_id = world_frame_id_;
    t.child_frame_id = body_frame_id_;

    // 填充平移
    t.transform.translation.x = state.p.x();
    t.transform.translation.y = state.p.y();
    t.transform.translation.z = state.p.z();

    // 填充旋转
    t.transform.rotation.w = state.q.w();
    t.transform.rotation.x = state.q.x();
    t.transform.rotation.y = state.q.y();
    t.transform.rotation.z = state.q.z();

    // 发送 TF
    tf_broadcaster_->sendTransform(t);
}

} // namespace uwb_imu_fusion