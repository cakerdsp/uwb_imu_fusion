#!/usr/bin/env python3
"""
ZUPT数据分析脚本
读取CSV文件并绘制加速度方差、陀螺仪模长和速度模长随时间的变化
"""

import os
import sys
import glob
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False

def find_latest_log_file():
    """查找最新的日志文件"""
    home_dir = os.path.expanduser("~")
    log_dir = os.path.join(home_dir, ".ros", "zupt_logs")
    
    if not os.path.exists(log_dir):
        print(f"日志目录不存在: {log_dir}")
        return None
    
    csv_files = glob.glob(os.path.join(log_dir, "zupt_data_*.csv"))
    if not csv_files:
        print(f"未找到日志文件: {log_dir}")
        return None
    
    latest_file = max(csv_files, key=os.path.getmtime)
    return latest_file

def plot_zupt_data(csv_file):
    """读取CSV文件并绘图"""
    try:
        df = pd.read_csv(csv_file)
    except Exception as e:
        print(f"读取文件失败: {e}")
        return
    
    if df.empty:
        print("文件为空")
        return
    
    if 'timestamp' not in df.columns:
        print("文件格式错误：缺少timestamp列")
        return
    
    # 计算相对时间（从第一个时间戳开始）
    df['relative_time'] = df['timestamp'] - df['timestamp'].iloc[0]
    
    # 创建图形
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    fig.suptitle('ZUPT触发条件分析', fontsize=16, fontweight='bold')
    
    # 绘制加速度方差
    axes[0].plot(df['relative_time'], df['acc_variance'], 'b-', linewidth=1, alpha=0.7)
    axes[0].set_ylabel('加速度方差 (m²/s⁴)', fontsize=12)
    axes[0].set_title('加速度方差随时间变化', fontsize=13)
    axes[0].grid(True, alpha=0.3)
    axes[0].axhline(y=0, color='k', linestyle='--', linewidth=0.5)
    
    # 绘制陀螺仪模长
    axes[1].plot(df['relative_time'], df['gyro_norm'], 'g-', linewidth=1, alpha=0.7)
    axes[1].set_ylabel('陀螺仪模长 (rad/s)', fontsize=12)
    axes[1].set_title('陀螺仪模长随时间变化', fontsize=13)
    axes[1].grid(True, alpha=0.3)
    axes[1].axhline(y=0, color='k', linestyle='--', linewidth=0.5)
    
    # 绘制速度模长
    axes[2].plot(df['relative_time'], df['vel_norm'], 'r-', linewidth=1, alpha=0.7)
    axes[2].set_ylabel('速度模长 (m/s)', fontsize=12)
    axes[2].set_xlabel('相对时间 (s)', fontsize=12)
    axes[2].set_title('速度模长随时间变化', fontsize=13)
    axes[2].grid(True, alpha=0.3)
    axes[2].axhline(y=0, color='k', linestyle='--', linewidth=0.5)
    
    plt.tight_layout()
    
    # 保存图片
    output_file = csv_file.replace('.csv', '_plot.png')
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"图表已保存到: {output_file}")
    
    # 显示统计信息
    print("\n=== 数据统计 ===")
    print(f"总数据点数: {len(df)}")
    print(f"时间范围: {df['relative_time'].iloc[-1]:.2f} 秒")
    print(f"\n加速度方差:")
    print(f"  最小值: {df['acc_variance'].min():.6f}")
    print(f"  最大值: {df['acc_variance'].max():.6f}")
    print(f"  平均值: {df['acc_variance'].mean():.6f}")
    print(f"  中位数: {df['acc_variance'].median():.6f}")
    print(f"\n陀螺仪模长:")
    print(f"  最小值: {df['gyro_norm'].min():.6f}")
    print(f"  最大值: {df['gyro_norm'].max():.6f}")
    print(f"  平均值: {df['gyro_norm'].mean():.6f}")
    print(f"  中位数: {df['gyro_norm'].median():.6f}")
    print(f"\n速度模长:")
    print(f"  最小值: {df['vel_norm'].min():.6f}")
    print(f"  最大值: {df['vel_norm'].max():.6f}")
    print(f"  平均值: {df['vel_norm'].mean():.6f}")
    print(f"  中位数: {df['vel_norm'].median():.6f}")
    
    # 显示图表
    plt.show()

def main():
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
        if not os.path.exists(csv_file):
            print(f"文件不存在: {csv_file}")
            return
    else:
        csv_file = find_latest_log_file()
        if csv_file is None:
            print("用法: python3 plot_zupt_data.py [csv_file_path]")
            print("或者不指定参数，将自动查找最新的日志文件")
            return
    
    print(f"读取文件: {csv_file}")
    plot_zupt_data(csv_file)

if __name__ == "__main__":
    main()

