#include "uwb_imu_fusion/uwb_location_node.hpp"
#include <regex>

namespace uwb_imu_fusion {

UwbLocationNode::UwbLocationNode(const rclcpp::NodeOptions& options)
    : Node("Uwb_Location", rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true)) {
    
    RCLCPP_INFO(this->get_logger(), "Starting Uwb_Location Node.");

    // 1. 加载参数 (YAML)
    load_parameters();

    // 2. 初始化算法模块
    // 此时仅使用占位符，后续根据 algo_type_ 实例化具体算法
    fusion_algo_ = std::make_unique<DummyAlgo>(); 
    
    // 初始化算法状态 (重力 g 已在 NavState 构造函数中默认设为 -9.81)
    NavState init_state;
    fusion_algo_->initialize(init_state);

    // 3. 初始化硬件
    init_hardware();

    // 4. 初始化 ROS 通信
    auto qos = rclcpp::SensorDataQoS();
    
    // 从参数获取话题名称
    std::string imu_topic = this->get_parameter("topics.imu_sub").as_string();
    std::string odom_topic = this->get_parameter("topics.odom_pub").as_string();

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, qos, 
        std::bind(&UwbLocationNode::imu_callback, this, std::placeholders::_1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // 5. 启动定时器 (50Hz / 20ms) 用于串口轮询
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
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port. Check permissions/connection.");
    }
}

void UwbLocationNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    ImuMeasurement imu;
    // 使用 IMU 消息自带的时间戳
    imu.timestamp = rclcpp::Time(msg->header.stamp).seconds();
    imu.acc << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
    imu.gyro << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;

    // 仅传入 acc 和 gyro 进行预测，不传入 orientation，确保不进行错误的姿态更新
    fusion_algo_->addImuData(imu);
    NavState current_state = fusion_algo_->getCurrentState();
    publish_tf(current_state, current_ros_time);
}

void UwbLocationNode::timer_callback() {
    // 隐式同步：使用当前接收时间作为测量时间
    rclcpp::Time current_ros_time = this->now();
    double current_ts = current_ros_time.seconds();
    // 读取并解析数据
    auto frames = serial_reader_.read_and_parse();
    if (frames.empty()) return;
    auto frame = frames.back();

    for (const auto& anchor_data : frame.anchors) {
        // 过滤未知的基站 ID
        if (anchors_.find(anchor_data.id) == anchors_.end() || anchor_data.q_value > nlos_q_threshold_) continue;

        UwbMeasurement meas;
        meas.timestamp = current_ts;
        meas.anchor_id = anchor_data.id;
        meas.anchor_pos = anchors_[anchor_data.id];
        meas.dist = anchor_data.dist;
        meas.q_value = anchor_data.q_value;

        // 执行更新。若状态发生显著变化（更新成功），发布里程计
        fusion_algo_->addUwbData(meas);
    }
    NavState current_state = fusion_algo_->getCurrentState();
    publish_odometry(current_state,current_ros_time);
    publish_tf(current_state, current_ros_time);
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