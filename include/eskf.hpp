#pragma once

#include "uwb_imu_fusion/data_types.hpp"
#include <Eigen/Dense>
#include <iostream>

namespace uwb_imu_fusion {

class ESKF : public FusionInterface {
public:
    ESKF();
    ~ESKF() = default;

    void initialize(const NavState& init_state) override;
    void addImuData(const ImuMeasurement& imu) override;
    bool addUwbData(const UwbMeasurement& uwb) override;
    NavState getCurrentState() override;

private:
    void predict(const ImuMeasurement& imu);
    void update(const UwbMeasurement& uwb);

private:
    bool initialized_ = false;
    double last_imu_time_ = 0.0;

    NavState state_;

    // 顺序: Error[ Pos(3), Vel(3), Att(3), AccBias(3), GyroBias(3), Gravity(3) ]
    Eigen::Matrix<double, 18, 18> P_;

    // 噪声矩阵 Q 保持 12x12 (重力通常没有过程噪声，或极小)
    // 噪声源: [n_acc, n_gyro, n_ba, n_bg]
    Eigen::Matrix<double, 12, 12> Q_;

    double R_uwb_;

    struct Config {
        double acc_noise_std = 0.1;
        double gyro_noise_std = 0.05;
        double acc_bias_walk_std = 1e-4;
        double gyro_bias_walk_std = 1e-5;
        double uwb_noise_std = 0.15;
    } config_;
};

} // namespace uwb_imu_fusion