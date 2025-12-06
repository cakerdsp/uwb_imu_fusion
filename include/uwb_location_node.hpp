#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

// 包含之前写好的模块
#include "serial_reader.hpp"   
#include "fusion_interface.hpp" 

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <chrono>

namespace uwb_imu_fusion {

class UwbLocationNode : public rclcpp::Node {
public:
    explicit UwbLocationNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~UwbLocationNode() = default;

private:
    // --- 核心流程函数 ---
    void load_parameters();
    void init_hardware();
    
    // --- 回调函数 ---
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void timer_callback(); // 串口轮询 & 主逻辑

    // --- 辅助函数 ---
    void publish_odometry(const NavState& state, const rclcpp::Time& stamp);
    void publish_tf(const NavState& state, const rclcpp::Time& stamp);

    // --- 成员变量 ---
    
    // 1. 算法核心 (多态指针，支持 ESKF/Graph 切换)
    std::unique_ptr<FusionInterface> fusion_algo_;

    // 2. 硬件驱动
    SerialReader serial_reader_;

    // 3. ROS 通信
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 4. 配置参数
    std::string world_frame_id_;
    std::string body_frame_id_;
    std::string algo_type_; // "ESKF" or "Dummy"
    double nlos_q_threshold_;  // NLOS 判定阈值
    
    // 基站映射表: Anchor ID -> [x, y, z]
    // 例如: 0 -> [0.0, 0.0, 2.5]
    std::map<int, Eigen::Vector3d> anchors_;
};

} // namespace uwb_imu_fusion