#include "uwb_location_node.hpp"
#include "initializer.hpp" // 引入初始化工具


namespace uwb_imu_fusion {

UwbLocationNode::UwbLocationNode(const rclcpp::NodeOptions& options)
    : Node("uwb_imu_fusion", rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true)) {
    
    RCLCPP_INFO(this->get_logger(), "Starting uwb_imu_fusion Node (Industrial FSM Version).");

    // 1. 加载参数
    load_parameters();

    // 2. 初始化算法 (先给个 Dummy 占位，真正初始化在 STATIC_INIT 结束时)
    if (algo_type_ == "eskf") {
        fusion_algo_ = std::make_unique<ESKF>();
        Config config;
        config.acc_noise_std = this->get_parameter("eskf.acc_noise_std").as_double();
        config.gyro_noise_std = this->get_parameter("eskf.gyro_noise_std").as_double();
        config.acc_bias_walk_std = this->get_parameter("eskf.acc_bias_walk_std").as_double();
        config.gyro_bias_walk_std = this->get_parameter("eskf.gyro_bias_walk_std").as_double();
        config.uwb_noise_std = this->get_parameter("eskf.uwb_noise_std").as_double();
        config.nlos_q_threshold = this->get_parameter("NLOS.nlos_q_threshold").as_double();
        fusion_algo_->setConfig(config);
    } else {
        fusion_algo_ = std::make_unique<DummyAlgo>();
    }

    // 3. 初始化 ROS 通信
    auto qos = rclcpp::SensorDataQoS();
    std::string imu_topic = this->get_parameter("topics.imu_sub").as_string();
    std::string uwb_topic = this->get_parameter("topics.uwb_sub").as_string();
    std::string odom_topic = this->get_parameter("topics.odom_pub").as_string();

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, qos, 
        std::bind(&UwbLocationNode::imu_callback, this, std::placeholders::_1));

    uwb_sub_ = this->create_subscription<uwb_imu_fusion::msg::UWB>(
        uwb_topic, qos,
        std::bind(&UwbLocationNode::uwb_callback, this, std::placeholders::_1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // 4. 初始化状态为IDLE，等待传感器数据
    sys_state_ = SysState::IDLE;

    // 5. 启动主循环定时器 (用于发布Odometry和状态监控)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20), // 50Hz
        std::bind(&UwbLocationNode::timer_callback, this));

    // 6. 初始化可视化
    rclcpp::QoS latching_qos(1);
    latching_qos.transient_local();
    latching_qos.reliable();

    viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "uwb/visualization", latching_qos);

    // 启动低频定时器 (1Hz 足够了，因为基站是静态的)
    viz_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&UwbLocationNode::timer_viz_callback, this));
}

void UwbLocationNode::load_parameters() {
    auto declare_param = [this](const std::string& name, auto default_val) {
        if (!this->has_parameter(name)) {
            this->declare_parameter(name, default_val);
        }
    };
    declare_param("topics.imu_sub", "/imu/data");
    declare_param("topics.uwb_sub", "/uwb/data");
    declare_param("topics.odom_pub", "/odometry/filtered");
    declare_param("frames.world_frame_id", "map");
    declare_param("frames.body_frame_id", "base_link");
    declare_param("algorithm_type", "Dummy");
    declare_param("NLOS.nlos_q_threshold", 6.0);
    declare_param("eskf.acc_noise_std", 0.1);
    declare_param("eskf.gyro_noise_std", 0.05);
    declare_param("eskf.acc_bias_walk_std", 1e-4);
    declare_param("eskf.gyro_bias_walk_std", 1e-5);
    declare_param("eskf.uwb_noise_std", 0.15);

    world_frame_id_ = this->get_parameter("frames.world_frame_id").as_string();
    body_frame_id_ = this->get_parameter("frames.body_frame_id").as_string();
    algo_type_ = this->get_parameter("algorithm_type").as_string();
    nlos_q_threshold_ = this->get_parameter("NLOS.nlos_q_threshold").as_double();
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
// UWB 回调: 处理来自 uwb_pub_node 的 UWB 数据
// -------------------------------------------------------------------------
void UwbLocationNode::uwb_callback(const uwb_imu_fusion::msg::UWB::SharedPtr msg) {
    rclcpp::Time current_ros_time = msg->header.stamp;
    double current_ts = current_ros_time.seconds();

    // 状态机分流
    switch (sys_state_) {
        case SysState::IDLE:
            // 收到第一帧UWB数据，开始静止初始化
            sys_state_ = SysState::STATIC_INIT;
            static_init_start_time_ = current_ros_time;
            
            // 清空 Buffer
            init_imu_buf_.clear();
            init_uwb_buf_.clear();
            
            RCLCPP_INFO(this->get_logger(), "Sensors detected. Start STATIC INIT (2s). Please keep robot STILL!");
            // 继续处理这帧数据（fall through）
            [[fallthrough]];

        case SysState::STATIC_INIT:
            // 收集 UWB 数据用于初始化
            for (size_t i = 0; i < msg->anchor_ids.size() && i < msg->dists.size() && i < msg->q_values.size(); ++i) {
                int anchor_id = msg->anchor_ids[i];
                if (anchors_.find(anchor_id) == anchors_.end()) continue;
                
                UwbMeasurement meas;
                meas.timestamp = current_ts;
                meas.anchor_id = anchor_id;
                meas.anchor_pos = anchors_[anchor_id];
                meas.dist = msg->dists[i];
                meas.q_value = msg->q_values[i];
                init_uwb_buf_.push_back(meas);
            }
            break;

        case SysState::RUNNING_FUSION:
            // 处理 UWB 数据 (Update)
            for (size_t i = 0; i < msg->anchor_ids.size() && i < msg->dists.size() && i < msg->q_values.size(); ++i) {
                int anchor_id = msg->anchor_ids[i];
                if (anchors_.find(anchor_id) == anchors_.end()) continue;
                if (msg->q_values[i] > nlos_q_threshold_) {
                    RCLCPP_WARN(this->get_logger(), "Anchor %d quality is too low. Skipping update.", anchor_id);
                    continue; // 质量过滤
                }
                UwbMeasurement meas;
                meas.timestamp = current_ts;
                meas.anchor_id = anchor_id;
                meas.anchor_pos = anchors_[anchor_id];
                meas.dist = msg->dists[i];
                meas.q_value = msg->q_values[i];

                // 执行更新
                fusion_algo_->addUwbData(meas);
            }
            // 喂狗
            last_uwb_time_ = current_ros_time;
            break;

        default:
            break;
    }
}

// -------------------------------------------------------------------------
// 定时器回调: 低频 (50Hz) - 状态流转 + Odom发布
// -------------------------------------------------------------------------
void UwbLocationNode::timer_callback() {
    rclcpp::Time current_ros_time = this->now();

    // 状态机主逻辑
    switch (sys_state_) {
        
        // -----------------------------------------------------
        // 阶段 0: 异常
        // -----------------------------------------------------
        case SysState::SYSTEM_ERROR:
            return;

        // -----------------------------------------------------
        // 阶段 1: IDLE (等待传感器数据就绪)
        // -----------------------------------------------------
        case SysState::IDLE:
            // 等待UWB回调触发状态转换
            break;

        // -----------------------------------------------------
        // 阶段 2: STATIC_INIT (静止初始化)
        // -----------------------------------------------------
        case SysState::STATIC_INIT: {
            // 检查时间是否足够 (2秒)
            if ((current_ros_time - static_init_start_time_).seconds() > 2.0) {
                
                // 检查数据量是否足够
                if (init_imu_buf_.size() < 50) {
                    RCLCPP_WARN(this->get_logger(), "Not enough IMU data for init. Retrying...");
                    static_init_start_time_ = current_ros_time; // 重置计时
                    return;
                }

                // --- 核心初始化逻辑 ---
                RCLCPP_INFO(this->get_logger(), "Calculating initial state...");

                // 1. 算出初始状态 (Roll, Pitch, Bias)
                NavState init_state = Initializer::alignIMU(init_imu_buf_);
                
                // 2. 算出初始位置 (调用你的三边定位库)
                init_state.p = Initializer::solveTrilateration(init_uwb_buf_, this->get_logger());
                
                // 3. 补上时间戳
                init_state.timestamp = current_ros_time.seconds();

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
            // 检查UWB超时 (1秒没数据 -> 警告)
            if ((current_ros_time - last_uwb_time_).seconds() > 1.0) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "UWB Lost! Continuing with IMU-only prediction.");
            }

            // 发布 Odometry (TF 已经在 imu_callback 发了)
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


void UwbLocationNode::timer_viz_callback() {
    // 如果没有订阅者，就不消耗 CPU 去构建消息
    if (viz_pub_->get_subscription_count() == 0) {
        return;
    }

    visualization_msgs::msg::MarkerArray msg;
    rclcpp::Time now = this->now();

    // 遍历所有基站
    for (auto const& [id, pos] : anchors_) {
        // 1. 基站球体 (Sphere)
        visualization_msgs::msg::Marker sphere;
        sphere.header.frame_id = world_frame_id_;
        sphere.header.stamp = now;
        sphere.ns = "anchors";
        sphere.id = id;
        sphere.type = visualization_msgs::msg::Marker::SPHERE;
        sphere.action = visualization_msgs::msg::Marker::ADD;
        
        sphere.pose.position.x = pos.x();
        sphere.pose.position.y = pos.y();
        sphere.pose.position.z = pos.z();
        sphere.pose.orientation.w = 1.0;

        sphere.scale.x = 0.2; // 20cm 直径
        sphere.scale.y = 0.2;
        sphere.scale.z = 0.2;

        sphere.color.r = 0.0f;
        sphere.color.g = 1.0f; // 绿色
        sphere.color.b = 0.0f;
        sphere.color.a = 0.8f;
        
        // 永久存在 (直到节点关闭)
        sphere.lifetime = rclcpp::Duration::from_seconds(0); 

        msg.markers.push_back(sphere);

        // 2. 基站文字标签 (Text)
        visualization_msgs::msg::Marker text;
        text.header = sphere.header;
        text.ns = "anchor_labels";
        text.id = id;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;

        text.pose.position = sphere.pose.position;
        text.pose.position.z += 0.3; // 文字显示在球体上方

        text.scale.z = 0.2; // 文字高度
        text.color.r = 1.0f;
        text.color.g = 1.0f;
        text.color.b = 1.0f;
        text.color.a = 1.0f;

        text.text = "A" + std::to_string(id);
        text.lifetime = rclcpp::Duration::from_seconds(0);

        msg.markers.push_back(text);
    }

    viz_pub_->publish(msg);
}
} // namespace uwb_imu_fusion