#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "uwb_imu_fusion/msg/uwb.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "fusion_interface.hpp" // 包含接口
#include "eskf.hpp"           // 包含实现

namespace uwb_imu_fusion {

class UwbLocationNode : public rclcpp::Node {
public:
    explicit UwbLocationNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~UwbLocationNode() = default;

private:
    void load_parameters();
    
    // 回调函数
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void uwb_callback(const uwb_imu_fusion::msg::UWB::SharedPtr msg);
    void timer_callback();

    // 发布函数
    void publish_odometry(const NavState& state, const rclcpp::Time& stamp);
    void publish_tf(const NavState& state, const rclcpp::Time& stamp);

private:
    // --- 状态机定义 ---
    enum class SysState {
        UNINITIALIZED,      // 刚启动
        IDLE,               // 硬件OK，等待数据
        STATIC_INIT,        // 静止初始化 (2秒)
        RUNNING_FUSION,     // 正常融合
        SYSTEM_ERROR        // 故障
    };
    SysState sys_state_ = SysState::UNINITIALIZED;

    // --- 初始化专用缓冲区 ---
    std::vector<ImuMeasurement> init_imu_buf_;
    std::vector<UwbMeasurement> init_uwb_buf_;
    rclcpp::Time static_init_start_time_;

    // --- 运行时监控 ---
    rclcpp::Time last_uwb_time_; // 看门狗

    // --- ROS 组件 ---
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<uwb_imu_fusion::msg::UWB>::SharedPtr uwb_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    // --- 模块 ---
    std::unique_ptr<FusionInterface> fusion_algo_;

    // --- 参数 ---
    std::map<int, Eigen::Vector3d> anchors_;
    std::string world_frame_id_;
    std::string body_frame_id_;
    std::string algo_type_;
    double nlos_q_threshold_;


    // 可视化相关
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
    rclcpp::TimerBase::SharedPtr viz_timer_;

    // 专门用于发布基站的可视化
    void timer_viz_callback();
};

} // namespace uwb_imu_fusion