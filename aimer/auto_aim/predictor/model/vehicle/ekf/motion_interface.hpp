/**
 * @file motion_interface.hpp
 * @brief 运动模型抽象接口
 *
 * 所有整车运动模型 (SpinMotion, LmtdMotion, SpMotion) 的公共基类
 * 提供统一的接口，消除 VehicleModel 中的 if-else 分支
 *
 * 设计原则:
 * - 接口尽可能通用，覆盖所有模型的共有功能
 * - 模型特有功能通过虚函数提供默认实现
 */

#ifndef __AIMER_AUTO_AIM_PREDICTOR_MOTION_MOTION_INTERFACE_HPP__
#define __AIMER_AUTO_AIM_PREDICTOR_MOTION_MOTION_INTERFACE_HPP__

#include <string>
#include <vector>

#include <Eigen/Core>

#include "aimer/auto_aim/predictor/types.hpp"
#include "aimer/auto_aim/predictor/observer/armor_tracker.hpp"

namespace autoaim::predictor {

/**
 * @brief 运动模型抽象接口
 *
 * 所有整车旋转模型必须实现此接口
 */
class MotionInterface {
public:
    virtual ~MotionInterface() = default;

    // ==================== 生命周期 ====================

    /**
     * @brief 初始化模型
     * @param armor 初始装甲板观测
     * @param timestamp 时间戳
     */
    virtual void init(const ArmorData& armor, double timestamp) = 0;

    /**
     * @brief 更新模型 (单装甲板)
     * @param armor 装甲板观测
     * @param timestamp 时间戳
     */
    virtual void update(const ArmorData& armor, double timestamp) = 0;

    /**
     * @brief 更新模型 (多装甲板)
     * @param armors 装甲板观测列表 (按 z_to_v 排序，正对优先)
     * @param timestamp 时间戳
     */
    virtual void update(const std::vector<ArmorData>& armors, double timestamp) = 0;

    /**
     * @brief 重置模型
     */
    virtual void reset() = 0;

    /**
     * @brief 模型是否有效 (已初始化)
     */
    virtual bool valid() const = 0;

    // ==================== 预测 ====================

    /**
     * @brief 预测旋转中心位置
     * @param dt 预测时间差 (秒)
     * @return 预测的旋转中心位置 (世界坐标系)
     */
    virtual Eigen::Vector3d predict_center(double dt) const = 0;

    /**
     * @brief 预测指定装甲板位置
     * @param armor_idx 装甲板索引 (0 = 当前追踪的装甲板)
     * @param dt 预测时间差 (秒)
     * @return 预测的装甲板位置 (世界坐标系)
     */
    virtual Eigen::Vector3d predict_armor_pos(int armor_idx, double dt) const = 0;

    // ==================== 状态查询 ====================

    /**
     * @brief 获取旋转中心速度
     * @return 中心速度 (m/s)
     */
    virtual Eigen::Vector3d get_velocity() const = 0;

    /**
     * @brief 获取当前追踪装甲板位置
     * @return 装甲板位置 (世界坐标系)
     */
    virtual Eigen::Vector3d get_armor_pos() const = 0;

    /**
     * @brief 获取当前追踪装甲板朝向角 (OUTWARD)
     * @return 角度 (rad)
     */
    virtual double get_theta() const = 0;

    /**
     * @brief 获取角速度
     * @return 角速度 (rad/s), 正值为逆时针
     */
    virtual double get_omega() const = 0;

    /**
     * @brief 获取当前半径 (或基础半径)
     * @return 半径 (m)
     */
    virtual double get_radius() const = 0;

    /**
     * @brief 获取另一个半径 (4装甲板时的长/短轴半径)
     * @return 另一个半径 (m)
     */
    virtual double get_another_radius() const = 0;

    /**
     * @brief 获取高度差 (当前装甲板与下一块装甲板的 z 差)
     * @return 高度差 (m)
     */
    virtual double get_dz() const = 0;

    /**
     * @brief 获取当前追踪的装甲板 ID
     * @return 装甲板 ID (0-3)
     */
    virtual int get_tracked_id() const = 0;

    // ==================== 可视化辅助 ====================

    /**
     * @brief 计算所有装甲板位置 (从 EKF 状态直接生成)
     * @param dt 预测时间差 (默认 0)
     * @return 所有装甲板位置列表, idx=0 是当前追踪的装甲板
     */
    virtual std::vector<Eigen::Vector3d> compute_all_armors(double dt = 0) const = 0;

    // ==================== PlotJuggler 输出 ====================

    /**
     * @brief 输出内部状态到 PlotJuggler
     * @param prefix 变量名前缀 (例如 "/target_1/vehicle")
     */
    virtual void output_to_plotter(const std::string& prefix) const = 0;

    // ==================== 模型信息 ====================

    /**
     * @brief 获取模型名称 (调试用)
     * @return 模型名称字符串 ("spin", "lmtd", "sp")
     */
    virtual const char* name() const = 0;

    /**
     * @brief 获取装甲板数量
     * @return 装甲板数量 (3 或 4)
     */
    virtual int armor_num() const = 0;
};

}  // namespace autoaim::predictor

#endif  // __AIMER_AUTO_AIM_PREDICTOR_MOTION_MOTION_INTERFACE_HPP__
