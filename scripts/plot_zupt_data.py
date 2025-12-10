#!/usr/bin/env python3
"""
ZUPT data analysis script.
Reads the CSV log and plots accelerometer variance, gyro norm, and velocity norm over time.
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
    """Find the latest log file"""
    home_dir = os.path.expanduser("~")
    log_dir = os.path.join(home_dir, ".ros", "zupt_logs")
    
    if not os.path.exists(log_dir):
        print(f"Log directory not found: {log_dir}")
        return None
    
    csv_files = glob.glob(os.path.join(log_dir, "zupt_data_*.csv"))
    if not csv_files:
        print(f"No log files found in: {log_dir}")
        return None
    
    latest_file = max(csv_files, key=os.path.getmtime)
    return latest_file

def plot_zupt_data(csv_file):
    """Read CSV file and plot"""
    try:
        df = pd.read_csv(csv_file)
    except Exception as e:
        print(f"Failed to read file: {e}")
        return
    
    if df.empty:
        print("File is empty")
        return
    
    if 'timestamp' not in df.columns:
        print("Invalid format: missing 'timestamp' column")
        return
    
    # relative time from first timestamp
    df['relative_time'] = df['timestamp'] - df['timestamp'].iloc[0]
    
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    fig.suptitle('ZUPT Trigger Condition Analysis', fontsize=16, fontweight='bold')
    
    axes[0].plot(df['relative_time'], df['acc_variance'], 'b-', linewidth=1, alpha=0.7)
    axes[0].set_ylabel('Accel variance (m²/s⁴)', fontsize=12)
    axes[0].set_title('Accel variance vs time', fontsize=13)
    axes[0].grid(True, alpha=0.3)
    axes[0].axhline(y=0, color='k', linestyle='--', linewidth=0.5)
    
    axes[1].plot(df['relative_time'], df['gyro_norm'], 'g-', linewidth=1, alpha=0.7)
    axes[1].set_ylabel('Gyro norm (rad/s)', fontsize=12)
    axes[1].set_title('Gyro norm vs time', fontsize=13)
    axes[1].grid(True, alpha=0.3)
    axes[1].axhline(y=0, color='k', linestyle='--', linewidth=0.5)
    
    axes[2].plot(df['relative_time'], df['vel_norm'], 'r-', linewidth=1, alpha=0.7)
    axes[2].set_ylabel('Velocity norm (m/s)', fontsize=12)
    axes[2].set_xlabel('Relative time (s)', fontsize=12)
    axes[2].set_title('Velocity norm vs time', fontsize=13)
    axes[2].grid(True, alpha=0.3)
    axes[2].axhline(y=0, color='k', linestyle='--', linewidth=0.5)
    
    plt.tight_layout()
    
    output_file = csv_file.replace('.csv', '_plot.png')
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"Plot saved to: {output_file}")
    
    print("\n=== Stats ===")
    print(f"Total samples: {len(df)}")
    print(f"Time span: {df['relative_time'].iloc[-1]:.2f} s")
    print(f"\nAccel variance:")
    print(f"  min: {df['acc_variance'].min():.6f}")
    print(f"  max: {df['acc_variance'].max():.6f}")
    print(f"  mean: {df['acc_variance'].mean():.6f}")
    print(f"  median: {df['acc_variance'].median():.6f}")
    print(f"\nGyro norm:")
    print(f"  min: {df['gyro_norm'].min():.6f}")
    print(f"  max: {df['gyro_norm'].max():.6f}")
    print(f"  mean: {df['gyro_norm'].mean():.6f}")
    print(f"  median: {df['gyro_norm'].median():.6f}")
    print(f"\nVelocity norm:")
    print(f"  min: {df['vel_norm'].min():.6f}")
    print(f"  max: {df['vel_norm'].max():.6f}")
    print(f"  mean: {df['vel_norm'].mean():.6f}")
    print(f"  median: {df['vel_norm'].median():.6f}")
    
    # show plot
    plt.show()

def main():
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
        if not os.path.exists(csv_file):
            print(f"File not found: {csv_file}")
            return
    else:
        csv_file = find_latest_log_file()
        if csv_file is None:
            print("Usage: python3 plot_zupt_data.py [csv_file_path]")
            print("Or run without args to load the latest log file")
            return
    
    print(f"Reading file: {csv_file}")
    plot_zupt_data(csv_file)

if __name__ == "__main__":
    main()

