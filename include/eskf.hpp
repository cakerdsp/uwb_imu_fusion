#pragma once

#include "uwb_imu_fusion/data_types.hpp"
#include <Eigen/Dense>
#include <iostream>

namespace uwb_imu_fusion {

class ESKF : public FusionInterface {
public:
    // 构造函数，支持传入配置参数（如噪声大小）
    ESKF();
    ~ESKF() = default;

    // --- 接口实现 ---
    void initialize(const NavState& init_state) override;
    void addImuData(const ImuMeasurement& imu) override;
    bool addUwbData(const UwbMeasurement& uwb) override;
    NavState getCurrentState() override;

    // --- 额外接口 ---
    bool isInitialized() const { return initialized_; }

private:
    // 核心步骤 1: 预测 (IMU 驱动, 处理高频)
    void predict(const ImuMeasurement& imu);

    // 核心步骤 2: 更新 (UWB 驱动, 处理低频)
    void update(const UwbMeasurement& uwb);

    // 工具函数：重置误差状态
    // (ESKF 的特点：每次更新完将误差注入名义状态后，误差清零)
    void resetErrorState();

private:
    // 系统状态标志
    bool initialized_ = false;
    double last_imu_time_ = 0.0;

    // 名义状态 (Nominal State)
    NavState state_;

    // 误差状态协方差矩阵 P (15x15)
    // 状态顺序: Error[ Position(3), Velocity(3), Attitude(3), AccBias(3), GyroBias(3) ]
    // 注意：Attitude Error 定义为局部切空间的角度误差 delta_theta
    Eigen::Matrix<double, 15, 15> P_;

    // 过程噪声协方差 Q (12x12)
    // 噪声源: [n_acc(3), n_gyro(3), n_bias_acc(3), n_bias_gyro(3)]
    Eigen::Matrix<double, 12, 12> Q_;

    // 观测噪声协方差 R (1x1) - UWB 测距噪声
    double R_uwb_;

    // 配置参数结构体 (硬编码或从外部 set)
    struct Config {
        // 连续时间噪声谱密度
        double acc_noise_std = 0.1;        // m/s^2 * 1/sqrt(Hz)
        double gyro_noise_std = 0.05;      // rad/s * 1/sqrt(Hz)
        double acc_bias_walk_std = 1e-4;   // m/s^3 * 1/sqrt(Hz)
        double gyro_bias_walk_std = 1e-5;  // rad/s^2 * 1/sqrt(Hz)
        
        // 观测噪声
        double uwb_noise_std = 0.15;       // m
    } config_;
};

} // namespace uwb_imu_fusion