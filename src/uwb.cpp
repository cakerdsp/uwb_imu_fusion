// src/uwb_driver_node.cpp
#include <rclcpp/rclcpp.hpp>
#include "serial_reader.hpp"
#include "uwb_imu_fusion/msg/uwb.hpp"
#include <algorithm>
#include <vector>
#include <numeric>

namespace uwb_imu_fusion {

class UwbDriverNode : public rclcpp::Node {
public:
    UwbDriverNode(const rclcpp::NodeOptions& options) 
        : Node("uwb_pub_node", rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true)) {
        auto declare_param = [this](const std::string& name, auto default_val) {
            if (!this->has_parameter(name)) {
                this->declare_parameter(name, default_val);
            }
        };
        declare_param("port_name", "/dev/ttyUSB0");
        declare_param("baud_rate", 115200);
        declare_param("frame_id", "uwb_link");
        declare_param("topics.uwb_pub", "/uwb/data");

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

            // 填充基站数据并按照 q_value 升序排序
            std::vector<int> anchor_ids;
            std::vector<double> dists;
            std::vector<double> q_values;
            
            for (const auto& anchor : frame.anchors) {
                anchor_ids.push_back(anchor.id);
                dists.push_back(anchor.dist);
                q_values.push_back(anchor.q_value);
            }
            
            // 创建索引数组并按照 q_value 升序排序
            std::vector<size_t> indices(anchor_ids.size());
            std::iota(indices.begin(), indices.end(), 0);
            std::sort(indices.begin(), indices.end(), 
                     [&q_values](size_t i1, size_t i2) {
                         return q_values[i1] < q_values[i2];
                     });
            
            // 按照排序后的索引填充消息
            for (size_t idx : indices) {
                msg.anchor_ids.push_back(anchor_ids[idx]);
                msg.dists.push_back(dists[idx]);
                msg.q_values.push_back(q_values[idx]);
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