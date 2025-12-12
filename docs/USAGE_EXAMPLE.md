# 使用 robot_msgs 的代码示例

## 1. 在头文件中包含消息类型

```cpp
#include "robot_msgs/msg/robot_state.hpp"
#include "robot_msgs/msg/robot_command.hpp"
```

## 2. 订阅机器人状态话题

```cpp
// 在类的构造函数或初始化函数中
auto robot_state_sub = this->create_subscription<robot_msgs::msg::RobotState>(
    "/egg_wheel/robot_states",  // 话题名称
    10,                         // QoS队列深度
    [this](const robot_msgs::msg::RobotState::SharedPtr msg) {
        this->robot_state_callback(msg);
    });

// 回调函数
void robot_state_callback(const robot_msgs::msg::RobotState::SharedPtr msg) {
    // 遍历所有电机状态
    for (size_t i = 0; i < msg->motor_state.size(); ++i) {
        const auto& motor = msg->motor_state[i];
        
        RCLCPP_INFO(this->get_logger(), 
            "Motor[%zu]: q=%.3f, dq=%.3f, tau=%.3f", 
            i, motor.q, motor.dq, motor.tau_est);
        
        // 检测轮子关节（索引4和9）
        if (i == 4 || i == 9) {
            // 这是轮子关节
            double wheel_velocity = motor.dq;  // 轮速 (rad/s)
            double wheel_position = motor.q;   // 轮子位置 (rad)
            
            // 检测是否停止
            if (std::abs(wheel_velocity) < 0.01) {
                RCLCPP_INFO(this->get_logger(), "Wheel %zu is stopped", i);
            }
        }
    }
}
```

## 3. 发布机器人控制命令

```cpp
// 在类的构造函数中创建发布者
auto robot_cmd_pub = this->create_publisher<robot_msgs::msg::RobotCommand>(
    "/egg_wheel/robot_commands",  // 话题名称
    10);                          // QoS队列深度

// 发布命令
void publish_robot_command() {
    auto cmd_msg = std::make_shared<robot_msgs::msg::RobotCommand>();
    cmd_msg->motor_command.resize(10);  // 10个电机
    
    // 设置每个电机的命令
    for (size_t i = 0; i < 10; ++i) {
        cmd_msg->motor_command[i].q = 0.0;      // 目标位置
        cmd_msg->motor_command[i].dq = 0.0;     // 目标速度
        cmd_msg->motor_command[i].tau = 0.0;    // 目标力矩
        cmd_msg->motor_command[i].kp = 10.0;    // 位置刚度
        cmd_msg->motor_command[i].kd = 0.6;     // 速度阻尼
    }
    
    // 设置轮子关节的速度命令（索引4和9）
    cmd_msg->motor_command[4].dq = 1.0;  // 左轮速度
    cmd_msg->motor_command[9].dq = 1.0;  // 右轮速度
    
    robot_cmd_pub->publish(*cmd_msg);
}
```

## 4. 完整的节点示例

```cpp
#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/msg/robot_state.hpp"
#include "robot_msgs/msg/robot_command.hpp"

class RobotMonitorNode : public rclcpp::Node {
public:
    RobotMonitorNode() : Node("robot_monitor") {
        // 订阅状态
        state_sub_ = this->create_subscription<robot_msgs::msg::RobotState>(
            "/egg_wheel/robot_states", 10,
            std::bind(&RobotMonitorNode::state_callback, this, std::placeholders::_1));
        
        // 发布命令
        cmd_pub_ = this->create_publisher<robot_msgs::msg::RobotCommand>(
            "/egg_wheel/robot_commands", 10);
        
        RCLCPP_INFO(this->get_logger(), "Robot Monitor Node started");
    }

private:
    void state_callback(const robot_msgs::msg::RobotState::SharedPtr msg) {
        // 检查机器人是否停止
        bool is_stopped = true;
        for (const auto& motor : msg->motor_state) {
            if (std::abs(motor.dq) > 0.01) {
                is_stopped = false;
                break;
            }
        }
        
        if (is_stopped) {
            RCLCPP_INFO(this->get_logger(), "Robot is stopped");
        }
        
        // 获取轮子速度（索引4和9）
        if (msg->motor_state.size() > 9) {
            double left_wheel_vel = msg->motor_state[4].dq;
            double right_wheel_vel = msg->motor_state[9].dq;
            RCLCPP_INFO(this->get_logger(), 
                "Wheel velocities: L=%.3f, R=%.3f", 
                left_wheel_vel, right_wheel_vel);
        }
    }
    
    rclcpp::Subscription<robot_msgs::msg::RobotState>::SharedPtr state_sub_;
    rclcpp::Publisher<robot_msgs::msg::RobotCommand>::SharedPtr cmd_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotMonitorNode>());
    rclcpp::shutdown();
    return 0;
}
```

## 5. 消息字段说明

### RobotState 消息
- `motor_state[]`: 电机状态数组
  - `q`: 当前位置 (rad) - 来自编码器
  - `dq`: 当前速度 (rad/s) - 轮速信息 ⭐
  - `ddq`: 当前加速度 (rad/s²)
  - `tau_est`: 估计力矩 (N·m)
  - `cur`: 估计电流

### RobotCommand 消息
- `motor_command[]`: 电机命令数组
  - `q`: 目标位置 (rad)
  - `dq`: 目标速度 (rad/s)
  - `tau`: 目标力矩 (N·m)
  - `kp`: 位置刚度系数
  - `kd`: 速度阻尼系数

## 6. 话题名称

- **状态话题**: `/egg_wheel/robot_states` (RobotState)
- **命令话题**: `/egg_wheel/robot_commands` (RobotCommand)

## 7. 轮子关节索引

根据配置文件分析，轮子关节对应：
- **左轮**: 索引 4 (第5个电机)
- **右轮**: 索引 9 (第10个电机)

