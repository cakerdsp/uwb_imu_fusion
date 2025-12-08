// src/uwb_driver_node.cpp
#include <rclcpp/rclcpp.hpp>
#include "serial_reader.hpp"
#include "uwb_imu_fusion/msg/uwb.hpp" // 生成的消息头文件

namespace uwb_imu_fusion {

class UwbDriverNode : public rclcpp::Node {
public:
    UwbDriverNode(const rclcpp::NodeOptions& options) 
        : Node("uwb_pub_node", rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true)) {
        
        // 1. 声明参数（如果配置文件中没有，使用默认值）
        this->declare_parameter("port_name", "/dev/ttyUSB0");
        this->declare_parameter("baud_rate", 115200);
        this->declare_parameter("frame_id", "uwb_link");
        this->declare_parameter("topics.uwb_pub", "/uwb/data");

        std::string port = this->get_parameter("port_name").as_string();
        int baud = this->get_parameter("baud_rate").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();

        // 2. 初始化串口
        if (!serial_.open(port, baud)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s", port.c_str());
            // 这里可以选择抛异常或者退出，但在构造函数里最好只是报错
        } else {
            RCLCPP_INFO(this->get_logger(), "Serial opened: %s @ %d", port.c_str(), baud);
        }

        // 3. 创建发布者
        std::string topic = this->get_parameter("topics.uwb_pub").as_string();
        pub_ = this->create_publisher<uwb_imu_fusion::msg::UWB>(topic, 10);

        // 4. 创建高频定时器 (例如 100Hz，甚至更高，取决于你对延迟的要求)
        // 这里的频率决定了“去串口缓冲区捞数据”的频率
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), 
            std::bind(&UwbDriverNode::poll_serial, this));
    }

private:
    void poll_serial() {
        // 读取并解析数据 (调用你之前写好的 read_and_parse)
        auto frames = serial_.read_and_parse();

        // 遍历解析出的每一帧，转换为 ROS 消息并发布
        for (const auto& frame : frames) {
            auto msg = uwb_imu_fusion::msg::UWB();
            
            // 填充 Header
            msg.header.stamp = this->now(); // 这里打上 ROS 接收时间戳
            msg.header.frame_id = frame_id_;
            
            msg.tag_id = frame.tag_id;

            // 填充基站数据
            for (const auto& anchor : frame.anchors) {
                msg.anchor_ids.push_back(anchor.id);
                msg.dists.push_back(anchor.dist);
                msg.q_values.push_back(anchor.q_value);
            }

            // 填充 IMU 数据 (如果有)
            msg.acc.x = frame.imu.acc.x;
            msg.acc.y = frame.imu.acc.y;
            msg.acc.z = frame.imu.acc.z;
            
            msg.gyro.x = frame.imu.gyro.x;
            msg.gyro.y = frame.imu.gyro.y;
            msg.gyro.z = frame.imu.gyro.z;

            // 发布
            pub_->publish(msg);
        }
    }

    SerialReader serial_;
    rclcpp::Publisher<uwb_imu_fusion::msg::UWB>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string frame_id_;
};

} // namespace

// 注册组件 (可选) 或者直接写 main
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<uwb_imu_fusion::UwbDriverNode>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}