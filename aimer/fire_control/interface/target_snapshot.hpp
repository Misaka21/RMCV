/**
 * @file target_snapshot.hpp
 * @brief 目标快照接口 - 包含自身状态和多目标信息
 *
 * 设计目的:
 *   提供火控模块所需的完整场景信息
 *   包括: 自身状态、目标列表、时间戳等
 */

#ifndef __AIMER_FIRE_CONTROL_INTERFACE_TARGET_SNAPSHOT_HPP__
#define __AIMER_FIRE_CONTROL_INTERFACE_TARGET_SNAPSHOT_HPP__

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "aimer/fire_control/interface/target_interface.hpp"

namespace fire_control {

/**
 * @brief 自身状态 (火控计算所需)
 *
 * 简化版的 RobotState，只包含火控必需的字段
 * 避免 fire_control 模块依赖 aimer::RobotState 的具体实现
 */
struct SelfState {
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();  // 云台姿态
    double yaw = 0;         // 云台 yaw (rad)
    double pitch = 0;       // 云台 pitch (rad)
    double bullet_speed = 15.0;  // 弹速 (m/s)

    // 底盘状态 (动打动)
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();

    /**
     * @brief 从四元数更新 yaw/pitch
     */
    void update_from_quaternion() {
        // ZYX 欧拉角
        Eigen::Vector3d euler = orientation.toRotationMatrix().eulerAngles(2, 1, 0);
        yaw = euler[0];
        pitch = euler[1];
    }
};

/**
 * @brief 目标快照接口
 *
 * 提供某一时刻的场景信息，供火控模块使用
 */
class TargetSnapshotInterface {
public:
    virtual ~TargetSnapshotInterface() = default;

    // ==================== 时间戳 ====================

    /**
     * @brief 图像采集时间戳 (s)
     */
    virtual double timestamp() const = 0;

    /**
     * @brief 预测完成时间戳 (s)
     */
    virtual double predict_timestamp() const = 0;

    // ==================== 自身状态 ====================

    /**
     * @brief 获取自身状态
     */
    virtual const SelfState& self_state() const = 0;

    // ==================== 目标信息 ====================

    /**
     * @brief 主目标 (已选中的目标)
     * @return 主目标指针，无目标时返回 nullptr
     */
    virtual const TargetInterface* primary_target() const = 0;

    /**
     * @brief 目标数量
     */
    virtual int target_count() const { return primary_target() ? 1 : 0; }

    /**
     * @brief 按索引获取目标
     * @param index 目标索引 (0 ~ target_count()-1)
     * @return 目标指针，越界时返回 nullptr
     */
    virtual const TargetInterface* target_at(int index) const {
        return (index == 0) ? primary_target() : nullptr;
    }

    // ==================== 辅助方法 ====================

    /**
     * @brief 是否有有效目标
     */
    bool has_target() const { return primary_target() != nullptr; }

    /**
     * @brief 帧 ID
     */
    virtual int frame_id() const { return 0; }
};

}  // namespace fire_control

#endif  // __AIMER_FIRE_CONTROL_INTERFACE_TARGET_SNAPSHOT_HPP__
