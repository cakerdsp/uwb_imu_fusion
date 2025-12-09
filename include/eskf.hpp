#pragma once

#include "fusion_interface.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <deque>
#include <numeric>

namespace uwb_imu_fusion {

class ESKF : public FusionInterface {
public:
    ESKF();
    ~ESKF() = default;

    void initialize(const NavState& init_state) override;
    void addImuData(const ImuMeasurement& imu) override;
    bool addUwbData(const UwbMeasurement& uwb) override;
    void setConfig(const Config& config) override;
    NavState getCurrentState() override;

private:
    void predict(const ImuMeasurement& imu);
    void update(const UwbMeasurement& uwb);
    void updateZUPT(); // 零速修正 (Zero Velocity Update)
    void updateZIHR(); // 零积分航向 (Zero Integrated Heading Rate)
    double getYaw(const Eigen::Quaterniond& q);

private:
    bool initialized_ = false;
    double last_imu_time_ = 0.0;
    double static_yaw_ref_ = 0.0;
    bool last_is_stationary_ = false;
    std::deque<double> acc_buffer_;
    const int ACC_BUFFER_SIZE = 20;
    int info_count_ = 0;
    const int INFO_PRINT_INTERVAL = 50;

    NavState state_;

    // 顺序: Error[ Pos(3), Vel(3), Att(3), AccBias(3), GyroBias(3), Gravity(3) ]
    Eigen::Matrix<double, 15, 15> P_;

    // 噪声矩阵 Q 保持 12x12 (重力通常没有过程噪声，或极小)
    // 噪声源: [n_acc, n_gyro, n_ba, n_bg]
    Eigen::Matrix<double, 12, 12> Q_;

    double R_uwb_;

    Config config_;
};

} // namespace uwb_imu_fusion