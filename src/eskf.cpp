// nominal state:[p,v,q,b_a,b_w,g] 19*1
// error state:[dp,dv,dtheta,db_a,db_w,dg] 18*1
// dtheta 采用轴角向量的方法进行表示

#include "uwb_imu_fusion/eskf.hpp"
#include <cmath>

namespace uwb_imu_fusion {

// 辅助函数：计算反对称矩阵 (Skew-symmetric matrix)
// [v]x = [  0, -vz,  vy]
//        [ vz,   0, -vx]
//        [-vy,  vx,   0]
inline Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m << 0.0, -v.z(), v.y(),
         v.z(), 0.0, -v.x(),
         -v.y(), v.x(), 0.0;
    return m;
}

ESKF::ESKF() {
    // 1. 初始化协方差矩阵 P (初始不确定性)
    P_.setIdentity();
    P_.block<3, 3>(0, 0) *= 1.0;   // 位置误差
    P_.block<3, 3>(3, 3) *= 0.1;   // 速度误差
    P_.block<3, 3>(6, 6) *= 0.05;  // 姿态误差 (rad^2)
    P_.block<3, 3>(9, 9) *= 1e-3;  // 加计零偏
    P_.block<3, 3>(12, 12) *= 1e-4;// 陀螺零偏

    // 2. 初始化过程噪声矩阵 Q (对角阵)
    // 注意：这里先存标准差的平方，后续 predict 时会乘以 dt
    Q_.setZero();
    Q_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * std::pow(config_.acc_noise_std, 2);
    Q_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * std::pow(config_.gyro_noise_std, 2);
    Q_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * std::pow(config_.acc_bias_walk_std, 2);
    Q_.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * std::pow(config_.gyro_bias_walk_std, 2);

    // 3. UWB 观测噪声
    R_uwb_ = std::pow(config_.uwb_noise_std, 2);
}

void ESKF::initialize(const NavState& init_state) {
    state_ = init_state;
    last_imu_time_ = init_state.timestamp;
    
    // 重置 P 阵，防止之前的状态影响
    P_.setIdentity(); 
    // ... 可以根据需要再次微调 P 的初值 ...
    
    initialized_ = true;
    std::cout << "[ESKF] Initialized at t=" << state_.timestamp << std::endl;
}

NavState ESKF::getCurrentState() {
    return state_;
}

void ESKF::addImuData(const ImuMeasurement& imu) {
    if (!initialized_) return;

    // 简单的 dt 检查
    double dt = imu.timestamp - last_imu_time_;
    if (dt <= 0 || dt > 1.0) {
        // 数据乱序或丢包严重，重置时间戳
        last_imu_time_ = imu.timestamp;
        return; 
    }

    predict(imu);
    last_imu_time_ = imu.timestamp;
}

bool ESKF::addUwbData(const UwbMeasurement& uwb) {
    if (!initialized_) return false;

    // 简单的距离有效性检查
    if (uwb.dist <= 0) return false;

    update(uwb);
    return true;
}

// --------------------------------------------------------------------------------
// 核心逻辑：Predict (IMU 积分 + 协方差传播)
// --------------------------------------------------------------------------------
void ESKF::predict(const ImuMeasurement& imu) {
    double dt = imu.timestamp - last_imu_time_;
    double dt2 = dt * dt;

    // --- 1. 名义状态 (Nominal State) 递推 ---
    // 使用中值积分或欧拉积分。这里为了性能和稳定性，使用欧拉积分，
    // 假设 IMU 频率较高 (100Hz+)，误差可由 ESKF 修正。

    // 1.1 去除零偏
    Eigen::Vector3d acc_unbiased = imu.acc - state_.ba;
    Eigen::Vector3d gyro_unbiased = imu.gyro - state_.bg;

    // 1.2 姿态更新 (四元数运动学)
    // q_next = q_curr * Exp(w * dt)
    // 旋转矢量 angle_vec = w * dt
    Eigen::Vector3d angle_vec = gyro_unbiased * dt;
    Eigen::Quaterniond delta_q;
    double angle = angle_vec.norm();
    if (angle < 1e-8) {
        delta_q = Eigen::Quaterniond::Identity();
    } else {
        // Eigen 的 AngleAxis 构造函数参数为 (angle, axis)
        delta_q = Eigen::Quaterniond(Eigen::AngleAxisd(angle, angle_vec / angle));
    }
    // 更新姿态 (注意顺序，通常是局部扰动右乘)
    state_.q = (state_.q * delta_q).normalized();

    // 1.3 速度更新
    // v_next = v_curr + (R * a_body + g) * dt
    // 这里的 acc_unbiased 是比力，需要加上重力 g (在 NavState 中默认为 [0,0,-9.81])
    Eigen::Matrix3d R = state_.q.toRotationMatrix();
    Eigen::Vector3d acc_world = R * acc_unbiased + state_.g;
    state_.v += acc_world * dt;

    // 1.4 位置更新
    // p_next = p_curr + v_curr * dt + 0.5 * (R * a_body + g) * dt^2
    state_.p += state_.v * dt + 0.5 * acc_world * dt2; // 注意：此时 state_.v 已经是 v_next 吗？
    // 修正：应该用旧的 v 或平均 v。标准公式 p = p + v*dt + 0.5*a*dt^2 使用的是 v_curr
    // 由于上面 v 已经更新了，这里减回去一点更精确，或者调整代码顺序。
    // 为了代码清晰，调整顺序：
    // 正确顺序：先更 Pos，再更 Vel。
    // 但由于上面 v 已经加了 a*dt，所以 Pos 公式应变为: p += (v_old * dt + 0.5 * a * dt^2)
    // 或者利用新速度： p += v_new * dt - 0.5 * a * dt^2 
    // 这里做个简单修正：
    state_.p -= 0.5 * acc_world * dt2; // 因为 v 已经是 v_next 了，这里减回去变成平均速度积分

    // 更新时间戳
    state_.timestamp = imu.timestamp;

    // --- 2. 误差状态 (Error State) 协方差传播 ---
    // P = Fx * P * Fx^T + Fi * Q * Fi^T

    // 2.1 构造状态转移矩阵 Fx (15x15)
    // 状态顺序: dp, dv, dtheta, dba, dbg
    Eigen::Matrix<double, 15, 15> Fx = Eigen::Matrix<double, 15, 15>::Identity();

    // dp / dv = I * dt
    Fx.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;

    // dv / dtheta = -R * [a_body]x * dt
    Fx.block<3, 3>(3, 6) = -R * skew(acc_unbiased) * dt;
    
    // dv / dba = -R * dt
    Fx.block<3, 3>(3, 9) = -R * dt;

    // dtheta / dbg = -I * dt (简化版，假设小角度)
    // 严谨版是 -Jl(w*dt) * dt，但在误差状态很小时近似为 -I*dt
    Fx.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;

    // 2.2 构造噪声输入矩阵 Fi (15x12)
    Eigen::Matrix<double, 15, 12> Fi = Eigen::Matrix<double, 15, 12>::Zero();
    
    // v_noise = R * n_acc * dt ??? 不，通常建模为 v_dot = ... + n_acc
    // 所以对应 dv 部分是 I (在 body 系下噪声需乘 R，若噪声定义在 body 系)
    // 这里假设噪声定义在 Body 系
    // dv / n_acc
    Fi.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity(); // 或者是 R，取决于 Q 的定义。通常 Q 是 Body 系，这里不需要 R ??
    // 等等，速度是在 World 系更新的，噪声 n_acc 是在 Body 系。
    // dot_v = R(a-b-n) + g => dot_v = R*a - R*b - R*n + g
    // 所以噪声项是 -R * n_acc。由于是协方差，负号无关，但 R 有关。
    // 为简化计算，很多开源 ESKF 实现(如 MSCKF)假设 Fi 为 Identity 且 Q 已经在 World 系？不，这不对。
    // 正确做法：Fi.block(3,0) = R; 这样 Q 才是 Body 系噪声。
    // 但是！为了避免每次乘 R，也可以把 Fi 设为 I，但 Q 这里其实代表的是 "Projected Noise"。
    // 让我们采用标准做法：Fi 部分保持简单的 Identity 结构，
    // 而在计算 Fi * Q * Fi^T 时，我们手动处理 R。
    // 或者：直接将 Fi 定义为把噪声映射到状态导数上。
    
    Fi.block<3, 3>(3, 0) = R; // Velocity 受 Body 系加速度噪声影响
    Fi.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity(); // Angle 受 Body 系陀螺噪声影响 (这里近似 I)
    Fi.block<3, 3>(9, 6) = Eigen::Matrix3d::Identity(); // Ba 受随机游走影响
    Fi.block<3, 3>(12, 9) = Eigen::Matrix3d::Identity(); // Bg 受随机游走影响

    // 2.3 协方差更新
    // P = Fx * P * Fx^T + Fi * (Q * dt) * Fi^T
    // Q_ 是连续谱密度，乘以 dt 变为离散噪声协方差
    // 提示：Fi 比较稀疏，直接矩阵乘法可能浪费，但为了代码可读性，先直接乘。
    // 工业级优化：通常手写稀疏乘法，这里用 Eigen 优化也不会太慢。
    P_ = Fx * P_ * Fx.transpose() + Fi * (Q_ * dt) * Fi.transpose();
    
    // 保持对称性
    P_ = 0.5 * (P_ + P_.transpose());
}

// --------------------------------------------------------------------------------
// 核心逻辑：Update (UWB 观测)
// --------------------------------------------------------------------------------
void ESKF::update(const UwbMeasurement& uwb) {
    // 1. 计算预测距离 (h(x))
    Eigen::Vector3d p_tag = state_.p;
    Eigen::Vector3d p_anchor = uwb.anchor_pos;
    Eigen::Vector3d diff = p_tag - p_anchor;
    double dist_pred = diff.norm();

    // 除零保护
    if (dist_pred < 1e-3) return;

    // 2. 计算残差 (Innovation)
    // z = dist_meas
    // y = z - h(x)
    double residual = uwb.dist - dist_pred;

    // 3. 计算观测雅可比 H (1x15)
    // H = [dh/dp, dh/dv, dh/dtheta, ...]
    // 只有 dh/dp 非零
    // dh/dp = (p - anchor)^T / ||p - anchor||
    Eigen::Matrix<double, 1, 15> H = Eigen::Matrix<double, 1, 15>::Zero();
    H.block<1, 3>(0, 0) = diff.transpose() / dist_pred;

    // 4. 计算卡尔曼增益 K
    // K = P * H^T * (H * P * H^T + R)^-1
    // 这里的 (H*P*H^T + R) 是标量 (1x1 矩阵)
    double S = (H * P_ * H.transpose())(0, 0) + R_uwb_;
    
    // 简单的卡方检验 (Chi-Square Test) / 门控
    // residual^2 / S < threshold
    if (residual * residual > 16.0 * S) { // 4-sigma 剔除
        // std::cout << "[ESKF] UWB Outlier rejected. Res=" << residual << std::endl;
        return;
    }

    Eigen::VectorXd K = P_ * H.transpose() / S; // (15x1)

    // 5. 更新误差状态
    // dx = K * y
    Eigen::VectorXd delta_x = K * residual;

    // 6. 更新协方差
    // P = (I - KH) * P
    // 使用 Joseph form: P = (I-KH)P(I-KH)^T + KRK^T 可以保证正定性，但计算量大
    // 简化形式: P = P - K*H*P (更常用且快)
    // P -= K * H * P_; // 这种写法在 Eigen 中可能产生临时对象
    Eigen::Matrix<double, 15, 15> I = Eigen::Matrix<double, 15, 15>::Identity();
    P_ = (I - K * H) * P_;
    P_ = 0.5 * (P_ + P_.transpose()); // 强制对称

    // 7. 注入名义状态 (Injection)
    // p = p + dp
    state_.p += delta_x.segment<3>(0);
    // v = v + dv
    state_.v += delta_x.segment<3>(3);
    // q = q * Exp(dtheta)
    Eigen::Vector3d delta_theta = delta_x.segment<3>(6);
    // 小角度四元数更新
    Eigen::Quaterniond dq(1, 0.5 * delta_theta.x(), 0.5 * delta_theta.y(), 0.5 * delta_theta.z());
    state_.q = (state_.q * dq.normalized()).normalized();
    
    // 零偏更新
    state_.ba += delta_x.segment<3>(9);
    state_.bg += delta_x.segment<3>(12);

    // 8. 重置误差状态 (Reset)
    // 在 ESKF 中，每次 inject 之后，error state 就归零了。
    // 协方差矩阵 P 已经在第6步正确更新了（相对于新的名义状态的误差协方差）。
    // 理论上需要对 P 进行投影 G*P*G^T，但对于小误差 G 近似为 I，所以这步省略。
}

void ESKF::resetErrorState() {
    // 这是一个逻辑概念，实际上我们不需要显式维护一个 delta_x 成员变量
    // 因为每次 update 都是算出来立刻加进去，然后 delta_x 就没用了。
}

} // namespace uwb_imu_fusion