#pragma once
#include "fusion_interface.hpp"
#include "trilateration.h"
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
        // --- 第一步：数据清洗与平均 ---
        // 既然是初始化，我们希望利用 buffer 里的多帧数据来降噪
        // 使用 map 累加每个基站的距离： <anchor_id, {总距离, 计数}>
        std::map<int, std::pair<double, int>> dist_sums;
        // 记录基站坐标： <anchor_id, 坐标>
        std::map<int, Eigen::Vector3d> anchor_positions;

        for (const auto& meas : uwb_buffer) {
            // 过滤无效数据
            if (meas.anchor_id < 0 || meas.anchor_id >= MAX_AHCHOR_NUMBER) continue;
            if (meas.dist <= 0.0) continue;

            dist_sums[meas.anchor_id].first += meas.dist;
            dist_sums[meas.anchor_id].second++;
            
            // 假设同一 ID 的基站坐标是不变的，直接覆盖即可
            if (anchor_positions.find(meas.anchor_id) == anchor_positions.end()) {
                anchor_positions[meas.anchor_id] = meas.anchor_pos;
            }
        }

        // --- 第二步：填充旧库所需的数据结构 ---
        // 准备旧库需要的定长数组
        vec3d anchorArray[MAX_AHCHOR_NUMBER];
        int distanceArray[MAX_AHCHOR_NUMBER]; // 单位：毫米

        // 初始化数组 (重要：旧库通过判断 distanceArray[i] > 0 来识别有效基站)
        for(int i = 0; i < MAX_AHCHOR_NUMBER; i++) {
            distanceArray[i] = 0;
            anchorArray[i].x = 0;
            anchorArray[i].y = 0;
            anchorArray[i].z = 0;
        }

        int valid_anchors_count = 0;

        // 遍历 map 填充数组
        for (const auto& kv : dist_sums) {
            int id = kv.first;
            double sum = kv.second.first;
            int count = kv.second.second;

            if (count == 0) continue;

            // 计算平均距离 (米)
            double avg_dist_m = sum / count;

            // 1. 填充距离：转换为毫米 (mm)
            distanceArray[id] = static_cast<int>(avg_dist_m * 1000.0);

            // 2. 填充基站坐标
            if (anchor_positions.count(id)) {
                anchorArray[id].x = anchor_positions[id].x();
                anchorArray[id].y = anchor_positions[id].y();
                anchorArray[id].z = anchor_positions[id].z();
                valid_anchors_count++;
            }
        }

        // 检查有效基站数量，少于3个无法定位
        if (valid_anchors_count < 3) {
            std::cerr << "[Initializer] Not enough anchors for trilateration. Count: " 
                      << valid_anchors_count << std::endl;
            return Eigen::Vector3d::Zero();
        }

        // --- 第三步：调用旧库解算 ---
        vec3d best_solution;
        int result = GetLocation(&best_solution, anchorArray, distanceArray);

        // --- 第四步：结果转换与返回 ---
        if (result >= 0) {
            // 解算成功
            return Eigen::Vector3d(best_solution.x, best_solution.y, best_solution.z);
        } else {
            // 解算失败 (错误码见 trilateration.cpp 定义)
            std::cerr << "[Initializer] Trilateration failed with error code: " << result << std::endl;
            // 失败时返回零向量，或者可以考虑返回 Buffer 中所有基站坐标的中心点作为保底
            return Eigen::Vector3d::Zero();
        }
    }
};

} // namespace uwb_imu_fusion