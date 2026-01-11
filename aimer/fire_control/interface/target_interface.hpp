/**
 * @file target_interface.hpp
 * @brief 目标抽象接口 - 火控模块的核心抽象
 *
 * 设计目的:
 *   使火控模块独立于具体目标类型 (装甲板/能量机关等)
 *   通过适配器模式将具体类型转换为统一接口
 *
 * 使用方式:
 *   1. AutoAim: VehicleState → VehicleTargetAdapter → TargetInterface
 *   2. AutoBuff: BuffState → BuffTargetAdapter → TargetInterface
 */

#ifndef __AIMER_FIRE_CONTROL_INTERFACE_TARGET_INTERFACE_HPP__
#define __AIMER_FIRE_CONTROL_INTERFACE_TARGET_INTERFACE_HPP__

#include <cmath>

#include <Eigen/Core>

namespace fire_control {

/**
 * @brief 目标尺寸 (用于命中判断)
 */
struct TargetSize {
    double width = 0;   // 目标宽度 (m)
    double height = 0;  // 目标高度 (m)
};

/**
 * @brief 目标抽象接口
 *
 * 所有可被火控系统瞄准的目标都实现此接口
 * 包括: 装甲板、能量机关扇叶等
 */
class TargetInterface {
public:
    virtual ~TargetInterface() = default;

    // ==================== 必须实现 ====================

    /**
     * @brief 目标是否有效
     */
    virtual bool is_valid() const = 0;

    /**
     * @brief 目标 ID (用于跟踪和切换判断)
     */
    virtual int target_id() const = 0;

    /**
     * @brief 目标当前位置 (相机坐标系)
     */
    virtual Eigen::Vector3d position() const = 0;

    /**
     * @brief 目标速度 (用于火控插值)
     */
    virtual Eigen::Vector3d velocity() const = 0;

    /**
     * @brief 目标尺寸 (用于命中判断)
     */
    virtual TargetSize size() const = 0;

    /**
     * @brief 目标朝向角 (装甲板法向/扇叶朝向, rad)
     */
    virtual double orientation_angle() const = 0;

    /**
     * @brief 目标置信度 (0~1)
     */
    virtual double confidence() const = 0;

    // ==================== 可选: 旋转目标扩展 ====================

    /**
     * @brief 目标是否在旋转 (陀螺/能量机关)
     */
    virtual bool is_rotating() const { return false; }

    /**
     * @brief 是否高速旋转 (需要打旋转中心)
     */
    virtual bool is_high_speed_rotating() const { return false; }

    /**
     * @brief 角速度 (rad/s, 正值为逆时针)
     */
    virtual double angular_velocity() const { return 0; }

    /**
     * @brief 当前相位 (rad)
     */
    virtual double phase() const { return 0; }

    /**
     * @brief 旋转中心 (相机坐标系)
     */
    virtual Eigen::Vector3d rotation_center() const { return position(); }

    // ==================== 可选: 多子目标扩展 (装甲板/扇叶) ====================

    /**
     * @brief 子目标数量 (装甲板数/扇叶数)
     * 默认为 1 (单目标)
     */
    virtual int sub_target_count() const { return 1; }

    /**
     * @brief 预测某个子目标在 dt 秒后的位置
     * @param index 子目标索引 (0 ~ sub_target_count()-1)
     * @param dt 预测时间 (s)
     * @return 预测位置，索引无效时返回 rotation_center()
     */
    virtual Eigen::Vector3d predict_sub_target_position(int index, double dt) const {
        (void)index;
        return predict_position(dt);
    }

    /**
     * @brief 预测旋转中心在 dt 秒后的位置
     */
    virtual Eigen::Vector3d predict_center(double dt) const {
        return rotation_center() + velocity() * dt;
    }

    // ==================== 辅助方法 ====================

    /**
     * @brief 预测 dt 秒后的位置
     *
     * 默认实现: 线性预测
     * 旋转目标可以重写此方法使用圆周运动预测
     */
    virtual Eigen::Vector3d predict_position(double dt) const {
        if (is_rotating() && std::abs(angular_velocity()) > 0.1) {
            // 旋转预测 (简化版，假设 z 不变)
            Eigen::Vector3d center = rotation_center();
            Eigen::Vector3d offset = position() - center;
            double r = offset.head<2>().norm();
            double current_angle = std::atan2(offset.y(), offset.x());
            double new_angle = current_angle + angular_velocity() * dt;
            return Eigen::Vector3d(
                center.x() + r * std::cos(new_angle),
                center.y() + r * std::sin(new_angle),
                center.z() + offset.z()
            );
        }
        return position() + velocity() * dt;
    }

    /**
     * @brief 目标与视线的夹角 (越小越正对, rad)
     */
    virtual double z_to_view_angle() const { return 0; }

    /**
     * @brief 距离 (到相机)
     */
    double distance() const { return position().norm(); }
};

}  // namespace fire_control

#endif  // __AIMER_FIRE_CONTROL_INTERFACE_TARGET_INTERFACE_HPP__
