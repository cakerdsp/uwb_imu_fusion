#include "uwb_imu_fusion/serial_reader.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <algorithm>
#include <sstream>
#include <cstring> // for strerror

namespace uwb_imu_fusion {

SerialReader::SerialReader() : fd_(-1) {
    // 预分配缓冲区，避免频繁扩容
    buffer_.reserve(2048);
}

SerialReader::~SerialReader() {
    close();
}

bool SerialReader::open(const std::string& port_name, int baud_rate) {
    port_name_ = port_name;
    
    // 我使用 O_NDELAY 开启非阻塞模式，防止卡死主线程
    fd_ = ::open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ == -1) {
        std::cerr << "[SerialReader] Open error: " << strerror(errno) << std::endl;
        return false;
    }

    struct termios options;
    tcgetattr(fd_, &options);

    // 设置波特率 (简单处理常见波特率)
    speed_t baud;
    switch(baud_rate) {
        case 115200: baud = B115200; break;
        case 921600: baud = B921600; break;
        default:     baud = B115200; break;
    }
    cfsetispeed(&options, baud);
    cfsetospeed(&options, baud);

    // 8N1 配置
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag |= (CLOCAL | CREAD);

    // 原始模式 (Raw Mode)，禁止回显和信号处理
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    options.c_iflag &= ~(IXON | IXOFF | IXANY);

    tcsetattr(fd_, TCSANOW, &options);
    tcflush(fd_, TCIFLUSH); // 清空旧数据

    return true;
}

void SerialReader::close() {
    if (fd_ != -1) {
        ::close(fd_);
        fd_ = -1;
    }
}

std::vector<UwbDataFrame> SerialReader::read_and_parse() {
    std::vector<UwbDataFrame> frames;
    if (fd_ == -1) return frames;

    uint8_t temp_buf[1024];
    // 我尝试尽可能多地读取数据
    int n = ::read(fd_, temp_buf, sizeof(temp_buf));

    if (n > 0) {
        // 追加到成员缓冲区
        buffer_.insert(buffer_.end(), temp_buf, temp_buf + n);
    }

    // 处理缓冲区中的完整行
    while (true) {
        // 查找行尾 (协议规定 \r\n，我这里宽容处理，找 \n)
        auto it = std::find(buffer_.begin(), buffer_.end(), '\n');
        
        if (it == buffer_.end()) {
            // 没有换行符，说明数据还没收完，保留在 buffer 中等待下次
            break; 
        }

        // 提取一行字符串 (排除 \r)
        std::string line(buffer_.begin(), it);
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }

        // 解析这一行
        if (!line.empty()) {
            auto frame = parse_line(line);
            if (frame) {
                frames.push_back(*frame);
            }
        }

        // 从缓冲区移除已处理的数据 (包括 \n)
        buffer_.erase(buffer_.begin(), it + 1);
    }
    
    // 保护机制：如果缓冲区堆积过大(全是乱码)，强制清空
    if (buffer_.size() > 4096) {
        buffer_.clear();
    }

    return frames;
}

std::optional<UwbDataFrame> SerialReader::parse_line(const std::string& line) {
    if (line.empty()) return std::nullopt;

    // 简单分割
    auto tokens = split(line, ',');
    if (tokens.empty()) return std::nullopt;

    UwbDataFrame frame;
    
    // 扩展点：根据第一个字段判断协议
    if (tokens[0] == "mi") {
        if (parse_mi_protocol(tokens, frame)) {
            return frame;
        }
    } 
    // else if (tokens[0] == "other_fmt") { ... }

    return std::nullopt;
}

// 解析特定的 "mi" 格式
bool SerialReader::parse_mi_protocol(const std::vector<std::string>& tokens, UwbDataFrame& frame) {
    // 协议文档：总字段数 31 个 (下标 0 到 30)
    // 0:mi, 1:time, 2-17:dist/q, 18-20:acc, 21-23:gyro, 24-26:mag, 27-29:euler, 30:id
    // 旧代码检查 < 22 是因为旧协议短，现在必须检查完整长度
    if (tokens.size() < 31) return false;

    frame.raw_protocol = "mi";

    try {
        // 1. 解析时间
        frame.timestamp = std::stof(tokens[1]);

        // 2. 解析 8 个基站的数据对
        for (int i = 0; i < 8; ++i) {
            int dist_idx = 2 + i * 2;
            int q_idx = 3 + i * 2;

            // 【关键修正】参考旧代码逻辑，显式处理 "null"
            // 注意：旧代码是用 range < 0 判断，这里直接判字符串更准
            if (trim(tokens[dist_idx]) == "null" || trim(tokens[q_idx]) == "null") {
                continue; 
            }

            UwbAnchorData anchor;
            anchor.id = i; 
            anchor.dist = std::stof(tokens[dist_idx]);
            anchor.q_value = std::stof(tokens[q_idx]);
            anchor.is_valid = true;

            frame.anchors.push_back(anchor);
        }

        // 3. 解析 IMU 数据 (严格按照你的文档索引)
        // 旧代码里是 tokens[10] 开始，那是旧协议，千万别学它！
        // 文档：18:accX ...
        
        // 加速度 (m/s^2)
        frame.imu.acc.x = std::stof(tokens[18]);
        frame.imu.acc.y = std::stof(tokens[19]);
        frame.imu.acc.z = std::stof(tokens[20]);
        
        // 陀螺仪 (rad/s)
        frame.imu.gyro.x = std::stof(tokens[21]);
        frame.imu.gyro.y = std::stof(tokens[22]);
        frame.imu.gyro.z = std::stof(tokens[23]);

        // 磁力计 (uT) - 旧代码可能没有这个
        frame.imu.mag.x = std::stof(tokens[24]);
        frame.imu.mag.y = std::stof(tokens[25]);
        frame.imu.mag.z = std::stof(tokens[26]);

        // 姿态 (degrees) - 文档说是 27,28,29
        // 旧代码用的 19,20,21 是错的(或者是旧版的)
        frame.imu.euler.roll  = std::stof(tokens[27]);
        frame.imu.euler.pitch = std::stof(tokens[28]);
        frame.imu.euler.yaw   = std::stof(tokens[29]);

        // 4. 解析 ID
        // 【关键修正】使用 trim 去除 \r\n，否则字符串比较会失败
        frame.tag_id = trim(tokens[30]);

        return true;

    } catch (const std::exception& e) {
        // 捕获 std::stof 转换失败 (比如数据乱码)
        // std::cerr << "[SerialReader] Parse error: " << e.what() << std::endl;
        return false;
    }
}

std::vector<std::string> SerialReader::split(const std::string& s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

std::string SerialReader::trim(const std::string& str) {
    const std::string whitespace = " \t\r\n";
    const auto strBegin = str.find_first_not_of(whitespace);
    if (strBegin == std::string::npos) return ""; // 全是空格

    const auto strEnd = str.find_last_not_of(whitespace);
    const auto strRange = strEnd - strBegin + 1;

    return str.substr(strBegin, strRange);
}

} // namespace uwb_imu_fusion