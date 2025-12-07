#pragma once
#include "uwb_imu_fusion/fusion_interface.hpp"
#include <vector>
#include <numeric>
#include <cmath>
#include <Eigen/Dense>
#include <iostream>

namespace uwb_imu_fusion {

class Initializer {
public:
    // 1. IMU 静止对齐 (计算零偏 + Roll/Pitch)
    static NavState alignIMU(const std::vector<ImuMeasurement>& imu_buffer) {
        NavState state;
        if (imu_buffer.empty()) return state;

        Eigen::Vector3d acc_sum = Eigen::Vector3d::Zero();
        Eigen::Vector3d gyro_sum = Eigen::Vector3d::Zero();

        for (const auto& m : imu_buffer) {
            acc_sum += m.acc;
            gyro_sum += m.gyro;
        }

        Eigen::Vector3d acc_avg = acc_sum / imu_buffer.size();
        Eigen::Vector3d gyro_avg = gyro_sum / imu_buffer.size();

        // A. 设定初始零偏 (假设静止时陀螺仪输出即为零偏)
        state.bg = gyro_avg; 
        state.ba.setZero(); // 加速度计零偏通常难以在静止中与重力区分，设为0

        // B. 设定初始姿态 (利用重力方向对齐 Roll 和 Pitch)
        // 假设重力在导航系下是 [0, 0, -9.81] (Z轴向上为正，重力向下)
        // 也就是说，重力向量 g 指向 -Z。
        // 机体静止时，加速度计测量的比力 f = -g = [0, 0, 9.81] (在导航系下)
        // 即 R * f_body = [0, 0, 9.81]
        
        // 简单计算：
        double ax = acc_avg.x();
        double ay = acc_avg.y();
        double az = acc_avg.z();

        // 利用反正切计算欧拉角
        double roll = std::atan2(ay, az);
        double pitch = std::atan2(-ax, std::sqrt(ay*ay + az*az));
        double yaw = 0.0; // 【关键】Yaw 设为 0，交给 ESKF 大方差去收敛

        // 转四元数
        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

        state.q = yawAngle * pitchAngle * rollAngle;
        
        // C. 其他置零
        state.v.setZero();
        
        return state;
    }

    // 2. UWB 初始位置解算 (你的库接口预留)
    static Eigen::Vector3d solveTrilateration(const std::vector<UwbMeasurement>& uwb_buffer) {
        if (uwb_buffer.empty()) return Eigen::Vector3d::Zero();

        // TODO: 这里接入你的三边定位库
        // 建议逻辑：
        // 1. 遍历 uwb_buffer，提取出几组质量最好的测距数据
        // 2. 调用你的库进行解算
        // 3. 返回解算出的坐标
        
        // --- 临时占位逻辑：返回 (0,0,0) 或 平均值 ---
        // 实际使用时请替换
        std::cout << "[Initializer] Using placeholder for Trilateration (Please implement me!)" << std::endl;
        return Eigen::Vector3d::Zero(); 
    }
};

} // namespace uwb_imu_fusion