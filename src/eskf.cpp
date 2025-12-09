#include "eskf.hpp"
#include <cmath>

namespace uwb_imu_fusion {

// 辅助函数：反对称矩阵
inline Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m << 0.0, -v.z(), v.y(),
         v.z(), 0.0, -v.x(),
         -v.y(), v.x(), 0.0;
    return m;
}

ESKF::ESKF() {

}

void ESKF::initialize(const NavState& init_state) {
    state_ = init_state;
    last_imu_time_ = init_state.timestamp;
    state_.g << 0.0, 0.0, -9.81;
    // 初始化时，重置协方差
    P_.setIdentity();
    // 同样需要给重力初始方差，否则它不会收敛
    // P_.block<3, 3>(15, 15) *= 0.01; 
    
    initialized_ = true;
    std::cout << "[ESKF] Initialized!" << std::endl;
}

NavState ESKF::getCurrentState() {
    return state_;
}

void ESKF::addImuData(const ImuMeasurement& imu) {
    if (!initialized_) return;
    double dt = imu.timestamp - last_imu_time_;
    if (dt <= 0 || dt > 1.0) {
        last_imu_time_ = imu.timestamp;
        return; 
    }
    predict(imu);
    last_imu_time_ = imu.timestamp;
    // 零速修正和航向修正
    double acc_norm = imu.acc.norm();
    double gyro_norm = imu.gyro.norm();
    if(gyro_norm < config_.ZIHR_limit) {
        updateZIHR();
    }
    if(std::abs(acc_norm - 9.81) < config_.ZUPT_limit) {
        updateZUPT();
    }
}

bool ESKF::addUwbData(const UwbMeasurement& uwb) {
    if (!initialized_) return false;
    if (uwb.dist <= 0) return false;
    update(uwb);
    return true;
}

// --------------------------------------------------------------------------------
// Predict: 15维
// --------------------------------------------------------------------------------
void ESKF::predict(const ImuMeasurement& imu) {
    static_yaw_ref_ = getYaw(state_.q);
    double dt = imu.timestamp - last_imu_time_;
    double dt2 = dt * dt;

    // 1. 名义状态递推
    Eigen::Vector3d acc_unbiased = imu.acc - state_.ba;
    Eigen::Vector3d gyro_unbiased = imu.gyro - state_.bg;

    // 姿态更新
    Eigen::Vector3d angle_vec = gyro_unbiased * dt;
    Eigen::Quaterniond delta_q;
    double angle = angle_vec.norm();
    if (angle < 1e-8) delta_q = Eigen::Quaterniond::Identity();
    else delta_q = Eigen::Quaterniond(Eigen::AngleAxisd(angle, angle_vec / angle));
    state_.q = (state_.q * delta_q).normalized();

    // 速度更新 (注意这里用了 state_.g)
    Eigen::Matrix3d R = state_.q.toRotationMatrix();
    Eigen::Vector3d acc_world = R * acc_unbiased + state_.g; // g 是变量

    // 位置更新
    state_.p += state_.v * dt + 0.5 * acc_world * dt2; 
    state_.v += acc_world * dt;
    state_.timestamp = imu.timestamp;

    // --- 2. 误差状态递推 ---
    
    // 【修改点 3】 Fx 变为 15x15
    Eigen::Matrix<double, 15, 15> Fx = Eigen::Matrix<double, 15, 15>::Identity();

    // dp/dv
    Fx.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;

    // dv/dtheta = -R[a]x * dt
    Fx.block<3, 3>(3, 6) = -R * skew(acc_unbiased) * dt;
    
    // dv/dba = -R * dt
    Fx.block<3, 3>(3, 9) = -R * dt;

    // 因为 v_dot = ... + g，所以 delta_v_dot = ... + delta_g
    // Fx.block<3, 3>(3, 15) = Eigen::Matrix3d::Identity() * dt;

    Fx.block<3, 3>(6, 6) = delta_q.toRotationMatrix().transpose();

    // dtheta/dbg = -I * dt
    Fx.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;

    // 【修改点 4】 Fi 变为 15x12
    Eigen::Matrix<double, 15, 12> Fi = Eigen::Matrix<double, 15, 12>::Zero();
    Fi.block<3, 3>(3, 0) = R;   // v noise
    Fi.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity(); // theta noise
    Fi.block<3, 3>(9, 6) = Eigen::Matrix3d::Identity(); // ba noise
    Fi.block<3, 3>(12, 9) = Eigen::Matrix3d::Identity();// bg noise
    // 重力通常建模为常量 (noise=0)，所以 Fi 对应重力的部分为 0
    // Q_ 初始化，此时未放入时间间因子
    Q_.setZero();
    Q_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * std::pow(config_.acc_noise_std, 2);
    Q_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * std::pow(config_.gyro_noise_std, 2);
    Q_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * std::pow(config_.acc_bias_walk_std, 2);
    Q_.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * std::pow(config_.gyro_bias_walk_std, 2);

    // P 更新
    P_ = Fx * P_ * Fx.transpose() + Fi * (Q_ * dt) * Fi.transpose();
    P_ = 0.5 * (P_ + P_.transpose());
}

// --------------------------------------------------------------------------------
// Update: 15维
// --------------------------------------------------------------------------------
void ESKF::update(const UwbMeasurement& uwb) {
    Eigen::Vector3d p_tag = state_.p;
    Eigen::Vector3d p_anchor = uwb.anchor_pos;
    Eigen::Vector3d diff = p_tag - p_anchor;
    double dist_pred = diff.norm();

    if (dist_pred < 1e-3) return;

    double residual = uwb.dist - dist_pred;

    // 【修改点 5】 H 变为 1x15
    Eigen::Matrix<double, 1, 15> H = Eigen::Matrix<double, 1, 15>::Zero();
    H.block<1, 3>(0, 0) = diff.transpose() / dist_pred;
    // 重力对位置没有直接观测导数，所以 H 其他部分都是 0
    // 重力是通过 P 矩阵里的相关性 (off-diagonal terms) 间接更新的

    R_uwb_ = std::pow(config_.uwb_noise_std, 2) * (1 + (uwb.q_value - 6 > 0 ? uwb.q_value - 6 : 0)); // 根据 q 值调整观测噪声
    double S = (H * P_ * H.transpose())(0, 0) + R_uwb_;

    Eigen::VectorXd K = P_ * H.transpose() / S; // (15x1)

    Eigen::VectorXd delta_x = K * residual;

    // P 更新
    Eigen::Matrix<double, 15, 15> I = Eigen::Matrix<double, 15, 15>::Identity();
    Eigen::Matrix<double, 15, 15> I_KH = I - K * H;
    P_ = I_KH * P_ * I_KH.transpose();
    P_ += (K * K.transpose()) * R_uwb_;

    // 防御性编程，人为修正偏差
    // 对于IMU偏置，不会改变的过于离谱
    Eigen::Vector3d delta_ba = delta_x.segment<3>(9);
    if (delta_ba.norm() > config_.acc_bias_limit) {
        // 如果模长超过限制，就按比例缩小 (保留方向)
        // 缩放系数 = 限幅值 / 当前模长
        delta_x.segment<3>(9) *= (config_.acc_bias_limit / delta_ba.norm());
        
        // 可选：打印日志，看看是不是经常触发
        // std::cout << "Accel Bias update clipped!" << std::endl;
    }
    Eigen::Vector3d delta_bg = delta_x.segment<3>(12);
    if (delta_bg.norm() > config_.gyro_bias_limit) {
        delta_x.segment<3>(12) *= (config_.gyro_bias_limit / delta_bg.norm());
        // std::cout << "Gyro Bias update clipped!" << std::endl;
    }
    // --- 注入误差 ---
    state_.p += delta_x.segment<3>(0);
    state_.v += delta_x.segment<3>(3);
    
    Eigen::Vector3d delta_theta = delta_x.segment<3>(6);
    Eigen::Quaterniond dq(1, 0.5 * delta_theta.x(), 0.5 * delta_theta.y(), 0.5 * delta_theta.z());
    state_.q = (state_.q * dq.normalized()).normalized();
    
    state_.ba += delta_x.segment<3>(9);
    state_.bg += delta_x.segment<3>(12);

    // 【新增】 注入重力误差
    // state_.g += delta_x.segment<3>(15);
}

void ESKF::setConfig(const Config& config) {
    config_ = config;
}

void ESKF::updateZUPT() {
    // 1. 观测维度 = 3 (vx, vy, vz)
    Eigen::Vector3d residual = -state_.v; // 期望速度 0，实际 state_.v

    // 2. H 矩阵 (3x15)
    // 速度误差位于状态向量索引 3~5
    Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
    H.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

    // 3. R 矩阵 (3x3)
    // 这是一个强约束，给极小的噪声
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * 1e-4;

    // 4. EKF 更新
    Eigen::Matrix<double, 3, 3> S = H * P_ * H.transpose() + R;
    Eigen::Matrix<double, 15, 3> K = P_ * H.transpose() * S.inverse();
    Eigen::VectorXd delta_x = K * residual;

    // 更新 P
    Eigen::Matrix<double, 15, 15> I = Eigen::Matrix<double, 15, 15>::Identity();
    P_ = (I - K * H) * P_;

    // 5. 注入误差
    state_.p += delta_x.segment<3>(0);
    state_.v += delta_x.segment<3>(3);
    
    // 姿态也可能被微调 (因为速度误差可能来源于姿态倾斜)
    Eigen::Vector3d delta_theta = delta_x.segment<3>(6);
    Eigen::Quaterniond dq(1, 0.5 * delta_theta.x(), 0.5 * delta_theta.y(), 0.5 * delta_theta.z());
    state_.q = (state_.q * dq.normalized()).normalized();

    // 修正加速度计零偏 (Acc Bias) -> 这是 ZUPT 的主要副产品
    state_.ba += delta_x.segment<3>(9);
    state_.bg += delta_x.segment<3>(12);

    // std::cout << "[ZUPT] Vel corrected." << std::endl;
}

// ========================================================
// 2. ZIHR: 航向锁定 (只观测 Yaw)
// ========================================================
void ESKF::updateZIHR() {
    // 1. 观测维度 = 1 (Yaw)
    double curr_yaw = getYaw(state_.q);
    double residual_scalar = static_yaw_ref_ - curr_yaw;

    // 角度归一化 (-PI ~ PI)
    while (residual_scalar > M_PI) residual_scalar -= 2 * M_PI;
    while (residual_scalar < -M_PI) residual_scalar += 2 * M_PI;
    
    Eigen::VectorXd residual(1);
    residual(0) = residual_scalar;

    // 2. H 矩阵 (1x15)
    // Yaw 误差位于状态向量索引 8
    Eigen::Matrix<double, 1, 15> H = Eigen::Matrix<double, 1, 15>::Zero();
    H(0, 8) = 1.0;

    // 3. R 矩阵 (1x1)
    Eigen::Matrix<double, 1, 1> R;
    R(0, 0) = 1e-5; // 强约束

    // 4. EKF 更新
    // 注意这里 S 是标量，求逆就是取倒数，计算极快
    Eigen::Matrix<double, 1, 1> S = H * P_ * H.transpose() + R;
    Eigen::Matrix<double, 15, 1> K = P_ * H.transpose() * S.inverse();
    Eigen::VectorXd delta_x = K * residual;

    // 更新 P
    Eigen::Matrix<double, 15, 15> I = Eigen::Matrix<double, 15, 15>::Identity();
    P_ = (I - K * H) * P_;

    // 5. 注入误差
    state_.p += delta_x.segment<3>(0);
    state_.v += delta_x.segment<3>(3);
    
    // 姿态修正 (重点修 Yaw)
    Eigen::Vector3d delta_theta = delta_x.segment<3>(6);
    Eigen::Quaterniond dq(1, 0.5 * delta_theta.x(), 0.5 * delta_theta.y(), 0.5 * delta_theta.z());
    state_.q = (state_.q * dq.normalized()).normalized();

    // 修正陀螺仪零偏 (Gyro Bias) -> 这是 ZIHR 的主要副产品
    state_.ba += delta_x.segment<3>(9);
    state_.bg += delta_x.segment<3>(12);

    // std::cout << "[ZIHR] Yaw locked." << std::endl;
}

double ESKF::getYaw(const Eigen::Quaterniond& q) {
    return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 
                      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}
} // namespace uwb_imu_fusion