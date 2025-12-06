#include "rclcpp/rclcpp.hpp"
#include "uwb_imu_fusion/uwb_location_node.hpp"
// tomorrow:
// 完成eskf和图优化的融合算法模块
// 完成状态机算法
// 实机测试，最起码完成静止测试

int main(int argc, char* argv[]) {
    // 1. 初始化 ROS 2 上下文
    rclcpp::init(argc, argv);

    try {
        // 2. 创建节点实例
        // 使用 make_shared 是为了配合 rclcpp::spin
        auto node = std::make_shared<uwb_imu_fusion::UwbLocationNode>();

        // 3. 运行节点，开始处理回调 (Timer, Subscription)
        RCLCPP_INFO(node->get_logger(), "Node initialized successfully, spinning...");
        rclcpp::spin(node);

    } catch (const std::exception& e) {
        // 捕获标准异常 (如参数加载失败、串口打开失败等)
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Node crashed with exception: %s", e.what());
    } catch (...) {
        // 捕获未知异常
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Node crashed with unknown exception.");
    }

    // 4. 清理资源并退出
    rclcpp::shutdown();
    return 0;
}