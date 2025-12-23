/**
 * @file position_ekf_ypd.hpp
 * @brief YPD坐标系的位置速度滤波器
 *
 * 参考: rm.cv.fans 的 PositionEkf
 *
 * 状态向量 (6维): [yaw, vyaw, pitch, vpitch, dis, vdis]
 * 观测向量 (3维): [yaw, pitch, dis]
 *
 * 优势:
 * - YPD坐标系更适合描述旋转目标
 * - 角度直接滤波，避免笛卡尔坐标的非线性误差
 * - 使用Ceres自动微分，无需手动推导雅可比
 */

#ifndef AIMER_COMMON_FILTER_POSITION_EKF_YPD_HPP
#define AIMER_COMMON_FILTER_POSITION_EKF_YPD_HPP

#include <cmath>
#include <vector>

#include <Eigen/Dense>
#include <ceres/jet.h>

#include "aimer/common/filter/adaptive_ekf.hpp"
#include "aimer/common/math/math.hpp"

namespace filter {

// ============================================================================
// 预测/观测函数对象
// ============================================================================

/**
 * @brief 匀速预测函数 (CV模型)
 *
 * 状态: [yaw, vyaw, pitch, vpitch, dis, vdis]
 * 转移: pos' = pos + vel * dt, vel' = vel
 */
struct YpdCVPredict {
    double dt;

    explicit YpdCVPredict(double delta_t) : dt(delta_t) {}

    template<typename T>
    void operator()(const T x_in[6], T x_out[6]) const {
        // yaw' = yaw + vyaw * dt
        x_out[0] = x_in[0] + T(dt) * x_in[1];
        x_out[1] = x_in[1];

        // pitch' = pitch + vpitch * dt
        x_out[2] = x_in[2] + T(dt) * x_in[3];
        x_out[3] = x_in[3];

        // dis' = dis + vdis * dt
        x_out[4] = x_in[4] + T(dt) * x_in[5];
        x_out[5] = x_in[5];
    }
};

/**
 * @brief 直接观测函数 (YPD → YPD)
 *
 * 状态: [yaw, vyaw, pitch, vpitch, dis, vdis]
 * 观测: [yaw, pitch, dis]
 */
struct YpdDirectMeasure {
    template<typename T>
    void operator()(const T x[6], T y[3]) const {
        y[0] = x[0];  // yaw
        y[1] = x[2];  // pitch
        y[2] = x[4];  // dis
    }
};

/**
 * @brief XYZ状态 → YPD观测的转换函数
 *
 * 状态: [x, vx, y, vy, z, vz] (笛卡尔)
 * 观测: [yaw, pitch, dis] (球坐标)
 *
 * 用于在笛卡尔空间滤波但使用YPD观测的场景
 */
struct XyzToYpdMeasure {
    template<typename T>
    void operator()(const T x[6], T y[3]) const {
        // 提取位置
        T px = x[0];
        T py = x[2];
        T pz = x[4];

        // xyz → ypd
        y[0] = ceres::atan2(py, px);  // yaw
        y[1] = ceres::atan2(pz, ceres::sqrt(px * px + py * py));  // pitch
        y[2] = ceres::sqrt(px * px + py * py + pz * pz);  // dis
    }
};

// ============================================================================
// 角度处理工具
// ============================================================================

/**
 * @brief 将角度归一化到 (-π, π]
 */
inline double normalize_angle(double angle) {
    while (angle > M_PI)  angle -= 2 * M_PI;
    while (angle <= -M_PI) angle += 2 * M_PI;
    return angle;
}

/**
 * @brief 获取最近的等价角度
 *
 * 用于处理角度跨越±π的情况
 * 例: target=179°, current=-179° → 返回181°
 *
 * @param target 目标角度
 * @param current 当前角度
 * @param range 周期范围 (默认2π)
 */
inline double get_closest_angle(double target, double current, double range = 2 * M_PI) {
    double diff = target - current;
    while (diff > range / 2)  diff -= range;
    while (diff < -range / 2) diff += range;
    return current + diff;
}

// ============================================================================
// PositionEkfYpd 主类
// ============================================================================

/**
 * @brief YPD坐标系位置速度滤波器
 *
 * 状态: [yaw, vyaw, pitch, vpitch, dis, vdis]
 *
 * 特点:
 * - 直接在YPD空间滤波
 * - 自动处理角度跨越±π的问题
 * - 支持运行时参数配置
 */
class PositionEkfYpd {
public:
    using Ekf = AdaptiveEkf<6, 3>;
    using VectorX = Eigen::Matrix<double, 6, 1>;
    using VectorY = Eigen::Matrix<double, 3, 1>;
    using MatrixXX = Eigen::Matrix<double, 6, 6>;
    using MatrixYY = Eigen::Matrix<double, 3, 3>;

    /**
     * @brief 噪声参数
     */
    struct NoiseParams {
        // 过程噪声 (Q对角线)
        double q_yaw   = 0.01;   // yaw 过程噪声
        double q_vyaw  = 0.1;    // vyaw 过程噪声
        double q_pitch = 0.01;   // pitch 过程噪声
        double q_vpitch = 0.1;   // vpitch 过程噪声
        double q_dis   = 0.01;   // dis 过程噪声
        double q_vdis  = 0.1;    // vdis 过程噪声

        // 观测噪声 (R对角线)
        double r_yaw   = 0.01;   // yaw 观测噪声
        double r_pitch = 0.01;   // pitch 观测噪声
        double r_dis_1m = 0.01;  // 1米处的dis观测噪声 (随距离平方增长)
    };

    PositionEkfYpd() = default;

    /**
     * @brief 使用YPD观测初始化
     */
    void init(const math::YpdCoord& ypd, double t) {
        VectorX x0;
        x0 << ypd.yaw, 0, ypd.pitch, 0, ypd.dis, 0;
        ekf_.init(x0);
        t_ = t;
    }

    /**
     * @brief 使用XYZ位置初始化
     */
    void init(const Eigen::Vector3d& xyz, double t) {
        math::YpdCoord ypd = math::xyz_to_ypd(xyz);
        init(ypd, t);
    }

    /**
     * @brief 设置噪声参数
     */
    void set_noise(const NoiseParams& params) { noise_ = params; }

    /**
     * @brief 更新滤波器
     *
     * @param ypd 观测值 (YPD)
     * @param t 时间戳
     */
    void update(const math::YpdCoord& ypd, double t) {
        double dt = t - t_;
        if (dt <= 0) return;

        // 构建噪声矩阵
        MatrixXX Q = build_Q(dt);
        MatrixYY R = build_R(ypd.dis);

        // 预测
        YpdCVPredict predict_func(dt);
        ekf_.predict_forward(predict_func, Q);

        // 处理角度跨越
        VectorX x = ekf_.get_x();
        double yaw_close = get_closest_angle(ypd.yaw, x[0]);
        double pitch_close = get_closest_angle(ypd.pitch, x[2]);

        // 观测更新
        VectorY y;
        y << yaw_close, pitch_close, ypd.dis;

        YpdDirectMeasure measure_func;
        ekf_.update_forward(measure_func, y, R);

        // 归一化角度
        VectorX x_new = ekf_.get_x();
        x_new[0] = normalize_angle(x_new[0]);
        x_new[2] = normalize_angle(x_new[2]);
        ekf_.set_x(x_new);

        t_ = t;
    }

    /**
     * @brief 更新滤波器 (XYZ输入)
     */
    void update(const Eigen::Vector3d& xyz, double t) {
        math::YpdCoord ypd = math::xyz_to_ypd(xyz);
        update(ypd, t);
    }

    // ========================================================================
    // 预测接口
    // ========================================================================

    /**
     * @brief 预测YPD坐标
     */
    math::YpdCoord predict_ypd(double t) const {
        double dt = t - t_;
        YpdCVPredict predict_func(dt);
        auto res = ekf_.predict(predict_func);
        return math::YpdCoord(res.x_p[0], res.x_p[2], res.x_p[4]);
    }

    /**
     * @brief 预测YPD速度
     */
    math::YpdCoord predict_ypd_v(double t) const {
        double dt = t - t_;
        YpdCVPredict predict_func(dt);
        auto res = ekf_.predict(predict_func);
        return math::YpdCoord(res.x_p[1], res.x_p[3], res.x_p[5]);
    }

    /**
     * @brief 预测XYZ位置
     */
    Eigen::Vector3d predict_pos(double t) const {
        math::YpdCoord ypd = predict_ypd(t);
        return math::ypd_to_xyz(ypd);
    }

    /**
     * @brief 预测XYZ速度
     *
     * 从YPD速度转换到XYZ速度
     * v_xyz = J * v_ypd，其中J是ypd_to_xyz的雅可比矩阵
     */
    Eigen::Vector3d predict_vel(double t) const {
        math::YpdCoord ypd = predict_ypd(t);
        math::YpdCoord ypd_v = predict_ypd_v(t);

        // 计算雅可比矩阵 (ypd → xyz)
        double cy = std::cos(ypd.yaw), sy = std::sin(ypd.yaw);
        double cp = std::cos(ypd.pitch), sp = std::sin(ypd.pitch);
        double d = ypd.dis;

        // ∂xyz/∂ypd 雅可比矩阵
        Eigen::Matrix3d J;
        // ∂x/∂(yaw, pitch, dis)
        J(0, 0) = -d * cp * sy;           // ∂x/∂yaw
        J(0, 1) = -d * sp * cy;           // ∂x/∂pitch
        J(0, 2) = cp * cy;                // ∂x/∂dis

        // ∂y/∂(yaw, pitch, dis)
        J(1, 0) = d * cp * cy;            // ∂y/∂yaw
        J(1, 1) = -d * sp * sy;           // ∂y/∂pitch
        J(1, 2) = cp * sy;                // ∂y/∂dis

        // ∂z/∂(yaw, pitch, dis)
        J(2, 0) = 0;                      // ∂z/∂yaw
        J(2, 1) = d * cp;                 // ∂z/∂pitch
        J(2, 2) = sp;                     // ∂z/∂dis

        Eigen::Vector3d v_ypd(ypd_v.yaw, ypd_v.pitch, ypd_v.dis);
        return J * v_ypd;
    }

    // ========================================================================
    // 访问器
    // ========================================================================

    double get_t() const { return t_; }

    VectorX get_state() const { return ekf_.get_x(); }

    void set_state(const VectorX& x) { ekf_.set_x(x); }

private:
    /**
     * @brief 构建过程噪声矩阵Q
     */
    MatrixXX build_Q(double dt) const {
        MatrixXX Q = MatrixXX::Zero();
        // 位置噪声随dt增长，速度噪声不变
        Q(0, 0) = noise_.q_yaw * dt;
        Q(1, 1) = noise_.q_vyaw * dt;
        Q(2, 2) = noise_.q_pitch * dt;
        Q(3, 3) = noise_.q_vpitch * dt;
        Q(4, 4) = noise_.q_dis * dt;
        Q(5, 5) = noise_.q_vdis * dt;
        return Q;
    }

    /**
     * @brief 构建观测噪声矩阵R
     *
     * 距离噪声随距离平方增长
     */
    MatrixYY build_R(double dis) const {
        MatrixYY R = MatrixYY::Zero();
        R(0, 0) = noise_.r_yaw;
        R(1, 1) = noise_.r_pitch;
        R(2, 2) = noise_.r_dis_1m * dis * dis;  // 随距离平方增长
        return R;
    }

    Ekf ekf_;
    double t_ = 0;
    NoiseParams noise_;
};

}  // namespace filter

#endif  // AIMER_COMMON_FILTER_POSITION_EKF_YPD_HPP
