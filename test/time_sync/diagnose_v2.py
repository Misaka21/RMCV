#!/usr/bin/env python3
"""
分析时间同步标定的原始数据，诊断为什么标准差这么大

从日志文件读取相机和IMU样本，可视化世界坐标系下的点云分布
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def analyze_log(log_file):
    """
    从日志文件中提取相机点和IMU数据，分析世界坐标系下的点分布

    需要的日志格式：
    - 相机点: cam_time, x, y, z (camera系)
    - IMU数据: imu_time, yaw, pitch, roll
    """
    print(f"读取日志文件: {log_file}")

    # TODO: 解析日志文件
    # 这里需要你提供日志文件的格式

    print("\n建议：")
    print("1. 在 test_time_sync.cpp 中添加调试输出，保存所有采样点")
    print("2. 输出格式: cam_time, p_cam.x, p_cam.y, p_cam.z, imu_time, yaw, pitch, roll")
    print("3. 然后用这个脚本分析点云分布")

if __name__ == "__main__":
    print("=" * 70)
    print("时间同步标定诊断工具 v2")
    print("=" * 70)
    print()
    print("当前问题：优化后标准差仍然 32mm，几乎没有改善")
    print()
    print("可能原因：")
    print("1. 外参不准确 (R_camera2gimbal 或 R_gimbal2imubody)")
    print("2. 相机内参不准确")
    print("3. 目标不是静止的")
    print("4. PnP 解算不稳定")
    print()
    print("诊断步骤：")
    print()
    print("步骤 1: 检查外参")
    print("  运行: ./test_ground_plane")
    print("  晃动云台，观察地平面是否稳定")
    print("  - 如果地面抖动 → 外参错误")
    print("  - 如果地面稳定 → 外参正确")
    print()
    print("步骤 2: 检查目标稳定性")
    print("  目视确认装甲板/标定板是否绝对静止")
    print("  - 使用三脚架固定")
    print("  - 或贴在墙上")
    print()
    print("步骤 3: 检查 PnP 精度")
    print("  在 test_time_sync 中添加输出：")
    print("  - 每帧 PnP 解算的 reprojection error")
    print("  - p_cam 的 z 坐标（距离）")
    print("  - 如果距离抖动很大 → PnP 不稳定")
    print()
    print("步骤 4: 减少采样点，提高质量")
    print("  - 只保留距离 > 1m 的样本（近距离 PnP 不准）")
    print("  - 只保留 reprojection error < 1px 的样本")
    print()
