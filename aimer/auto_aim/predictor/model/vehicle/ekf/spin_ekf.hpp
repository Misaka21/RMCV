/**
 * @file spin_motion.hpp
 * @brief 整车旋转模型 - XYZ状态 + YPD观测 EKF
 *
 * 设计思路:
 * - 状态用 XYZ 直角坐标: 预测方程线性精确 (匀速运动)
 * - 观测用 YPD 球坐标: 噪声建模准确 (角度/距离解耦)
 * - 自动微分计算雅可比: 无需手动推导
 *
 * 状态向量 (10维, 参考 FYT2024):
 *   [xc, vx, yc, vy, zc, vz, θ, ω, r, dz]
 *   - xc, yc, zc: 旋转中心位置 (世界系)
 *   - vx, vy, vz: 旋转中心速度
 *   - θ: 当前装甲板朝向角 (从中心指向装甲板的方向, OUTWARD, rad)
 *   - ω: 角速度 (rad/s)
 *   - r: 当前装甲板半径
 *   - dz: 当前装甲板高度差 (za = zc + dz)
 *
 * 观测向量 (4维):
 *   [yaw_a, pitch_a, dis_a, θ_a]
 *   - yaw_a, pitch_a, dis_a: 装甲板位置的球坐标
 *   - θ_a: 装甲板朝向角 (= θ, OUTWARD)
 *
 * 几何关系 (OUTWARD):
 *   armor = center + r * (cos θ, sin θ)   (θ 从中心指向装甲板)
 *   center = armor - r * (cos θ, sin θ)   (从装甲板反推中心)
 *
 * 外部维护 (跳变时交换):
 *   - another_r: 另一个半径 (4装甲板车辆长短轴)
 *   - another_dz: 另一个高度差
 */

#ifndef __AIMER_AUTO_AIM_PREDICTOR_MOTION_SPIN_MOTION_HPP__
#define __AIMER_AUTO_AIM_PREDICTOR_MOTION_SPIN_MOTION_HPP__

#include <cmath>

#include <Eigen/Core>
#include <ceres/jet.h>

#include "aimer/auto_aim/predictor/types.hpp"
#include "aimer/auto_aim/predictor/observer/armor_tracker.hpp"
#include "aimer/common/filter/adaptive_ekf.hpp"
#include "motion_interface.hpp"

namespace autoaim::predictor {

// ============================================================================
// 常量定义
// ============================================================================

namespace spin_model {

constexpr int N_X = 10;  // 状态维度 (参考 FYT2024)
constexpr int N_Z = 4;   // 观测维度

// 状态索引 (参考 FYT2024: xc, vxc, yc, vyc, zc, vzc, yaw, vyaw, r, d_zc)
enum StateIdx {
    XC = 0,     // 旋转中心 X
    VX = 1,     // X 速度
    YC = 2,     // 旋转中心 Y
    VY = 3,     // Y 速度
    ZC = 4,     // 旋转中心 Z
    VZ = 5,     // Z 速度
    THETA = 6,  // 车体朝向角
    OMEGA = 7,  // 角速度
    R = 8,      // 当前半径
    DZ = 9      // 当前高度差 (za = zc + dz)
};

// 观测索引
enum ObsIdx {
    YAW = 0,        // 装甲板方位角
    PITCH = 1,      // 装甲板俯仰角
    DIS = 2,        // 装甲板距离
    ARMOR_YAW = 3   // 装甲板朝向角
};

}  // namespace spin_model

// ============================================================================
// 预测函数 (CV模型，线性)
// ============================================================================

/**
 * @brief 匀速预测函数
 *
 * 状态转移 (完全线性):
 *   xc' = xc + vx·dt
 *   vx' = vx
 *   ...
 *   θ' = θ + ω·dt
 *   ω' = ω
 *   r' = r
 *   dz' = dz  (高度差不变)
 */
struct SpinCVPredict {
    double dt;

    explicit SpinCVPredict(double delta_t) : dt(delta_t) {}

    template<typename T>
    void operator()(const T x_in[spin_model::N_X], T x_out[spin_model::N_X]) const {
        // 位置 += 速度 * dt
        x_out[spin_model::XC] = x_in[spin_model::XC] + T(dt) * x_in[spin_model::VX];
        x_out[spin_model::VX] = x_in[spin_model::VX];

        x_out[spin_model::YC] = x_in[spin_model::YC] + T(dt) * x_in[spin_model::VY];
        x_out[spin_model::VY] = x_in[spin_model::VY];

        x_out[spin_model::ZC] = x_in[spin_model::ZC] + T(dt) * x_in[spin_model::VZ];
        x_out[spin_model::VZ] = x_in[spin_model::VZ];

        // 朝向角 += 角速度 * dt
        x_out[spin_model::THETA] = x_in[spin_model::THETA] + T(dt) * x_in[spin_model::OMEGA];
        x_out[spin_model::OMEGA] = x_in[spin_model::OMEGA];

        // 半径和高度差不变
        x_out[spin_model::R] = x_in[spin_model::R];
        x_out[spin_model::DZ] = x_in[spin_model::DZ];
    }
};

// ============================================================================
// 观测函数 (XYZ状态 → YPD观测)
// ============================================================================

/**
 * @brief XYZ状态 → YPD观测
 *
 * 从旋转中心和朝向角计算装甲板位置，再转换为 YPD 球坐标
 *
 * 约定: θ 是装甲板朝向角 (从中心指向装甲板的方向, OUTWARD)
 *
 * 装甲板位置 (OUTWARD):
 *   xa = xc + r·cos(θ)
 *   ya = yc + r·sin(θ)
 *   za = zc + dz  (dz 现在来自状态向量)
 *
 * YPD 观测:
 *   yaw = atan2(ya, xa)
 *   pitch = atan2(za, √(xa² + ya²))
 *   dis = √(xa² + ya² + za²)
 *   armor_yaw = θ  (装甲板朝向角, OUTWARD)
 */
struct SpinMeasure {
    template<typename T>
    void operator()(const T x[spin_model::N_X], T y[spin_model::N_Z]) const {
        // 从状态提取
        T xc = x[spin_model::XC];
        T yc = x[spin_model::YC];
        T zc = x[spin_model::ZC];
        T theta = x[spin_model::THETA];
        T r = x[spin_model::R];
        T dz = x[spin_model::DZ];  // 从状态中获取 dz

        // 计算装甲板位置 (世界系, OUTWARD)
        T xa = xc + r * ceres::cos(theta);
        T ya = yc + r * ceres::sin(theta);
        T za = zc + dz;

        // 计算 YPD 观测
        T rho_sq = xa * xa + ya * ya;
        T rho = ceres::sqrt(rho_sq);
        T d = ceres::sqrt(rho_sq + za * za);

        y[spin_model::YAW] = ceres::atan2(ya, xa);
        y[spin_model::PITCH] = ceres::atan2(za, rho);
        y[spin_model::DIS] = d;
        y[spin_model::ARMOR_YAW] = theta;  // 装甲板朝向 = θ (OUTWARD)
    }
};

// ============================================================================
// SpinMotion - 整车旋转模型
// ============================================================================

/**
 * @brief 整车旋转模型
 *
 * 功能:
 * - 10维 EKF 滤波 (含 dz 在状态中)
 * - 装甲板跳变处理 (半径/高度交换)
 * - 陀螺等级判断
 * - 多装甲板位置预测
 */
class SpinMotion : public MotionInterface {
public:
    using Ekf = aimer::filter::AdaptiveEkf<spin_model::N_X, spin_model::N_Z>;
    using VectorX = Eigen::Matrix<double, spin_model::N_X, 1>;
    using VectorZ = Eigen::Matrix<double, spin_model::N_Z, 1>;
    using MatrixXX = Eigen::Matrix<double, spin_model::N_X, spin_model::N_X>;
    using MatrixZZ = Eigen::Matrix<double, spin_model::N_Z, spin_model::N_Z>;

    /**
     * @brief 构造
     * @param armor_num 装甲板数量 (3 或 4)
     */
    explicit SpinMotion(int armor_num = 4);

    // ==================== MotionInterface 实现 ====================

    void init(const ArmorData& armor, double timestamp) override;
    void update(const ArmorData& armor, double timestamp) override;
    void update(const std::vector<ArmorData>& armors, double timestamp) override;
    void reset() override;
    bool valid() const override { return initialized_; }

    Eigen::Vector3d predict_center(double dt) const override;
    Eigen::Vector3d predict_armor_pos(int armor_idx, double dt) const override;

    Eigen::Vector3d get_velocity() const override;
    Eigen::Vector3d get_armor_pos() const override;
    double get_theta() const override { return ekf_.get_x()[spin_model::THETA]; }
    double get_omega() const override { return ekf_.get_x()[spin_model::OMEGA]; }
    double get_radius() const override { return ekf_.get_x()[spin_model::R]; }
    double get_another_radius() const override { return another_r_; }
    double get_dz() const override { return ekf_.get_x()[spin_model::DZ]; }
    int get_tracked_id() const override { return tracked_state_id_; }

    std::vector<Eigen::Vector3d> compute_all_armors(double dt = 0) const override;
    void log_state(const std::string& prefix) const override;
    const char* name() const override { return "spin"; }
    int armor_num() const override { return armor_num_; }

    // ==================== 额外方法 ====================

    /**
     * @brief 获取另一个高度差 (外部维护)
     */
    double get_another_dz() const { return another_dz_; }

    /**
     * @brief 从观测装甲板反推所有装甲板位置 (兼容旧接口)
     * @deprecated 使用 compute_all_armors() 替代
     */
    std::vector<Eigen::Vector3d> compute_all_armors_from_observation(
        const Eigen::Vector3d& observed_pos,
        double observed_theta) const;

    /**
     * @brief 通知发生装甲板跳变 (由上层 VehicleModel 调用)
     * @deprecated 跳变检测已移到内部
     */
    void notify_jump(int jump_index, const ArmorData& new_armor);

    VectorX get_state() const { return ekf_.get_x(); }

private:
    /**
     * @brief 构建过程噪声矩阵
     */
    MatrixXX build_Q(double dt) const;

    /**
     * @brief 构建观测噪声矩阵
     * @param distance 装甲板距离
     * @param z_to_v 装甲板朝向与视线夹角 (越大越侧面)
     * @param observed_armor_count 本次观测到的装甲板数量 (1或2，影响朝向噪声)
     */
    MatrixZZ build_R(double distance, double z_to_v, int observed_armor_count = 1) const;

    /**
     * @brief 从观测构建重置状态
     *
     * 用于门限检查拒绝过多时重新初始化 EKF
     *
     * @param armor 当前观测的装甲板数据
     * @return 重置状态向量
     */
    VectorX build_reset_state(const ArmorData& armor) const;

    // ==================== EKF ====================
    Ekf ekf_;
    bool initialized_ = false;
    double last_update_time_ = 0;

    // ==================== 装甲板配置 ====================
    int armor_num_ = 4;         // 装甲板数量 (3 或 4)
    double another_r_ = 0.26;   // 另一个半径 (4装甲板时)
    double another_dz_ = 0;     // 另一个高度差 (dz 现在在状态向量中)

    // ==================== 追踪状态 ====================
    double last_yaw_ = 0;       // 上一帧观测的 armor_yaw (用于连续化)

    // ==================== 追踪状态 ====================
    // tracked_state_id_: 当前追踪的装甲板序号 (0..armor_num-1)，用于统一对外语义
    // tracked_detector_id_: ArmorIdentifier 分配的线程 ID，仅用于跨帧关联同一物理装甲板
    int tracked_state_id_ = 0;
    int tracked_detector_id_ = -1;

    /**
     * @brief 检测并处理装甲板跳变 (内部调用，在 predict 之后)
     *
     * @param armor 当前观测装甲板
     * @param out_tracked_state_id 输出: 新的追踪装甲板序号 (0..N-1)
     * @return 是否发生跳变
     */
    bool detect_and_handle_jump(const ArmorData& armor, int& out_tracked_state_id);
};

}  // namespace autoaim::predictor

#endif  // __AIMER_AUTO_AIM_PREDICTOR_MOTION_SPIN_MOTION_HPP__
