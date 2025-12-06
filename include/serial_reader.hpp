#pragma once

#include <string>
#include <vector>
#include <optional>
#include <cstdint>

namespace uwb_imu_fusion {

// 1. 定义数据结构
// 保持简单 POD 类型，方便后续转 Eigen 或 ROS 消息

// 单个基站的数据
struct UwbAnchorData {
    int id;           // 基站索引 (0-7)
    double dist;      // 距离 (m)
    double q_value;   // 信号质量 (dBm)，这是判断有效性的核心
    bool is_valid;    // 解析是否成功 (非 null)
};

// 备用的 IMU 数据 (目前存着备用)
struct RawImuData {
    struct { double x, y, z; } acc;   // m/s^2
    struct { double x, y, z; } gyro;  // rad/s
    struct { double x, y, z; } mag;   // uT
    struct { double roll, pitch, yaw; } euler; // degrees
};

// 完整的一帧数据
struct UwbDataFrame {
    double timestamp; // 模块内部时间 (s)
    std::string tag_id;
    std::vector<UwbAnchorData> anchors; // 有效的基站数据列表
    RawImuData imu;   // IMU 数据
    std::string raw_protocol; // 协议头，如 "mi"
};

// 2. 串口读取类
class SerialReader {
public:
    SerialReader();
    ~SerialReader();

    // 打开串口，配置波特率和非阻塞模式
    bool open(const std::string& port_name, int baud_rate);
    
    // 关闭串口
    void close();

    // 核心接口：读取并解析
    // 每次调用会读取缓冲区所有可用数据，并返回解析好的数据帧列表
    std::vector<UwbDataFrame> read_and_parse();

private:
    int fd_; // 文件描述符
    std::string port_name_;
    std::vector<uint8_t> buffer_; // 内部持久化缓冲区，处理断包/粘包

    // 我将解析逻辑拆分，方便你后续扩展其他协议
    std::optional<UwbDataFrame> parse_line(const std::string& line);
    
    // 专门处理 "mi" 格式
    bool parse_mi_protocol(const std::vector<std::string>& tokens, UwbDataFrame& frame);
    
    // 辅助：分割字符串
    std::vector<std::string> split(const std::string& s, char delimiter);
    std::string trim(const std::string& str);
};

} // namespace uwb_imu_fusion