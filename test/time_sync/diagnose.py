#!/usr/bin/env python3
"""
时间同步标定诊断工具

用法：
    python3 diagnose.py <相机样本数> <IMU样本数> <优化前std> <优化后std>

示例：
    python3 diagnose.py 150 1500 18.173 17.797
"""

import sys

def diagnose_calibration(cam_samples, imu_samples, initial_std, final_std):
    print("=" * 70)
    print("时间同步标定诊断报告")
    print("=" * 70)

    # 1. 数据量检查
    print("\n1. 数据量检查:")
    print(f"   相机样本数: {cam_samples}")
    print(f"   IMU样本数: {imu_samples}")

    if cam_samples < 50:
        print("   ❌ 相机样本不足 (建议 >= 100)")
    elif cam_samples < 100:
        print("   ⚠️  相机样本偏少 (建议 >= 100)")
    else:
        print("   ✅ 相机样本充足")

    if imu_samples < 500:
        print("   ⚠️  IMU样本偏少 (建议 >= 1000)")
    else:
        print("   ✅ IMU样本充足")

    # 2. 优化效果检查
    print("\n2. 优化效果检查:")
    print(f"   优化前标准差: {initial_std:.3f} mm")
    print(f"   优化后标准差: {final_std:.3f} mm")

    improvement = (initial_std - final_std) / initial_std * 100
    print(f"   改善率: {improvement:.1f}%")

    if improvement < 10:
        print("   ❌ 优化几乎无效！(<10% 改善)")
        print("      → 可能存在系统性误差，而非时间偏移问题")
    elif improvement < 50:
        print("   ⚠️  优化效果有限 (10%-50% 改善)")
    else:
        print("   ✅ 优化效果显著 (>50% 改善)")

    # 3. 最终精度评估
    print("\n3. 最终精度评估:")
    if final_std < 2:
        print(f"   ✅ 优秀 ({final_std:.3f} mm < 2 mm)")
    elif final_std < 5:
        print(f"   ⚠️  可用 ({final_std:.3f} mm, 建议 < 2 mm)")
    elif final_std < 10:
        print(f"   ❌ 较差 ({final_std:.3f} mm, 必须 < 5 mm)")
    else:
        print(f"   ❌ 失败 ({final_std:.3f} mm >> 5 mm)")

    # 4. 可能原因分析
    print("\n4. 失败原因分析:")

    if final_std > 10:
        print("\n   最可能的原因（按概率排序）：")
        print()
        print("   A. 外参不准确 (概率 70%)")
        print("      → R_camera2gimbal 或 R_gimbal2imubody 错误")
        print("      → 解决：先运行 1.test_gimbal2imubody 和 2.test_extrinsic_calib")
        print()
        print("   B. 目标不是静止的 (概率 20%)")
        print("      → 装甲板或支架在晃动")
        print("      → 解决：更换更稳固的支架，或贴墙角标定")
        print()
        print("   C. 检测精度问题 (概率 10%)")
        print("      → PnP 解算不稳定")
        print("      → 解决：改善光照，降低曝光时间，增加装甲板尺寸")

    elif final_std > 5:
        print("\n   可能原因：")
        print("   • 外参有小误差")
        print("   • 目标有轻微晃动")
        print("   • 云台晃动不够充分")

    # 5. 建议操作
    print("\n5. 建议操作:")

    if final_std > 10:
        print("\n   ⚠️  当前标定失败，请按以下步骤操作：")
        print()
        print("   步骤 1: 重新标定外参")
        print("      cd build")
        print("      ./1.test_gimbal2imubody  # 标定 R_gimbal2imubody")
        print("      ./2.test_extrinsic_calib # 标定 R_camera2gimbal")
        print()
        print("   步骤 2: 检查目标稳定性")
        print("      • 使用三脚架固定装甲板")
        print("      • 或选择墙角、门框等刚性目标")
        print()
        print("   步骤 3: 重新时间同步标定")
        print("      ./3.test_time_sync")
        print()
        print("   预期结果：优化后标准差 < 2 mm")

    elif final_std > 2:
        print("\n   当前精度可用但不理想，建议：")
        print("   • 增加采集时间（20秒）")
        print("   • 增大云台晃动幅度")
        print("   • 改善光照条件")

    else:
        print("\n   ✅ 标定成功！可以将结果写入配置文件")

    print("\n" + "=" * 70)


if __name__ == "__main__":
    if len(sys.argv) != 5:
        print(__doc__)
        sys.exit(1)

    cam_samples = int(sys.argv[1])
    imu_samples = int(sys.argv[2])
    initial_std = float(sys.argv[3])
    final_std = float(sys.argv[4])

    diagnose_calibration(cam_samples, imu_samples, initial_std, final_std)
