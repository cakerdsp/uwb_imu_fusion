# UWB-IMU 融合定位功能包

这是一个ROS2功能包，实现了IMU和UWB（超宽带）传感器的融合定位，使用ESKF（Error State Kalman Filter）算法进行传感器融合。

## 功能特性

- IMU数据订阅和处理
- UWB数据通过串口读取
- ESKF算法进行传感器融合
- 发布融合后的里程计和TF变换
- 支持参数配置和状态机管理

## 依赖

### ROS2依赖
- `rclcpp`
- `sensor_msgs`
- `nav_msgs`
- `geometry_msgs`
- `tf2_ros`
- `tf2`
- `tf2_geometry_msgs`

### 系统依赖
- `Eigen3` (矩阵运算库)
- `libserial-dev` (Linux串口库，仅Linux系统需要)

## 编译

1. 将功能包放置到ROS2工作空间的`src`目录下：
```bash
cd ~/your_ros2_ws/src
# 将功能包复制到这里
```

2. 安装依赖（Ubuntu/Debian）：
```bash
sudo apt-get update
sudo apt-get install -y \
  libeigen3-dev \
  libserial-dev \
  ros-<your-ros2-distro>-rclcpp \
  ros-<your-ros2-distro>-sensor-msgs \
  ros-<your-ros2-distro>-nav-msgs \
  ros-<your-ros2-distro>-geometry-msgs \
  ros-<your-ros2-distro>-tf2-ros \
  ros-<your-ros2-distro>-tf2 \
  ros-<your-ros2-distro>-tf2-geometry-msgs
```

3. 编译功能包：
```bash
cd ~/your_ros2_ws
colcon build --packages-select uwb_imu_fusion
source install/setup.bash
```

## 配置

编辑 `config/default.yaml` 文件来配置功能包参数：

- **ROS通信配置**：话题名称、坐标系名称
- **串口配置**：UWB设备串口端口、波特率
- **算法参数**：ESKF噪声参数、NLOS阈值
- **基站配置**：UWB基站的位置坐标

## 运行

### 使用launch文件启动（推荐）

```bash
ros2 launch uwb_imu_fusion uwb_imu_fusion.launch.py
```

### 使用自定义配置文件

```bash
ros2 launch uwb_imu_fusion uwb_imu_fusion.launch.py config_file:=/path/to/your/config.yaml
```

### 直接运行节点

```bash
ros2 run uwb_imu_fusion uwb_imu_fusion_node --ros-args --params-file config/default.yaml
```

## 话题

### 订阅
- `/imu/data` (sensor_msgs/Imu): IMU数据输入

### 发布
- `/odometry/filtered` (nav_msgs/Odometry): 融合后的里程计数据
- TF变换：从`world_frame_id`到`body_frame_id`的坐标变换

## 参数说明

主要参数（在config/default.yaml中配置）：

- `topics.imu_sub`: IMU话题名称
- `topics.odom_pub`: 里程计发布话题
- `frames.world_frame_id`: 世界坐标系名称（默认：map）
- `frames.body_frame_id`: 机器人本体坐标系名称（默认：base_link）
- `serial.port_name`: UWB串口设备路径（Linux：/dev/ttyUSB0）
- `serial.baud_rate`: 串口波特率（默认：115200）
- `algorithm_type`: 融合算法类型（eskf/graph）
- `anchors`: UWB基站位置配置

## 注意事项

1. **串口权限**：确保当前用户有权限访问串口设备：
   ```bash
   sudo usermod -a -G dialout $USER
   # 然后重新登录
   ```

2. **Windows系统**：当前代码主要针对Linux系统，在Windows上可能需要修改串口相关代码。

3. **基站配置**：确保配置的UWB基站位置与实际环境一致。

## 许可证

MIT License

