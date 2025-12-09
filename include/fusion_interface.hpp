#pragma once

#include <Eigen/Dense>

namespace uwb_imu_fusion {

// 定义的通用导航状态，输出给里程计
struct NavState {
    double timestamp;
    Eigen::Vector3d p;    // 位置
    Eigen::Vector3d v;    // 速度
    Eigen::Quaterniond q; // 姿态
    Eigen::Vector3d bg;   // 陀螺仪零偏
    Eigen::Vector3d ba;   // 加速度计零偏
    Eigen::Vector3d g;    // 重力 (新增)

    NavState() {
        timestamp = 0.0;
        p.setZero(); v.setZero(); bg.setZero(); ba.setZero();
        q.setIdentity();
        g << 0.0, 0.0, -9.81; // 默认重力指向 Z 轴负方向
    }
};

// 定义的测量数据结构
struct ImuMeasurement {
    double timestamp;
    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
};

struct UwbMeasurement {
    double timestamp;
    int anchor_id;
    Eigen::Vector3d anchor_pos;
    double dist;
    double q_value;
};

struct Config {
    // eskf
    double acc_noise_std = 0.1;
    double gyro_noise_std = 0.05;
    double acc_bias_walk_std = 1e-4;
    double gyro_bias_walk_std = 1e-5;
    double uwb_noise_std = 0.15;
    double nlos_q_threshold = 6;
    double acc_bias_limit = 0.1;
    double gyro_bias_limit = 0.1;
    // 【新增】Bias 限幅阈值
    double ZUPT_limit = 0.05; // 例如 0.05 m/s^2
    double ZIHR_limit = 0.01; // 例如 0.01 rad/s
};

// 设计的策略模式接口，兼容 ESKF 和 Graph
class FusionInterface {
public:
    virtual ~FusionInterface() = default;

    virtual void initialize(const NavState& init_state) = 0;
    virtual void addImuData(const ImuMeasurement& imu) = 0;
    
    // 处理 UWB 数据，如果状态更新了返回 true
    virtual bool addUwbData(const UwbMeasurement& uwb) = 0;

    virtual void setConfig(const Config& config) = 0;
    
    virtual NavState getCurrentState() = 0;
};

// 写的一个空实现，用于测试数据流
class DummyAlgo : public FusionInterface {
    NavState state_;
    Config config_;
public:
    void initialize(const NavState& init_state) override { state_ = init_state; }
    
    void addImuData(const ImuMeasurement& imu) override {
        state_.timestamp = imu.timestamp;
    }
    
    bool addUwbData(const UwbMeasurement& uwb) override {
        state_.timestamp = uwb.timestamp;
        return true; // 假装更新了
    }
    
    void setConfig(const Config& config) override { config_ = config; }
    
    NavState getCurrentState() override { return state_; }
};

} // namespace uwb_imu_fusion