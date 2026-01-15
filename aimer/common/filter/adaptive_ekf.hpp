/**
 * @file adaptive_ekf.hpp
 * @brief 基于Ceres自动微分的扩展卡尔曼滤波器
 *
 * 参考: rm.cv.fans 的 AdaptiveEkf
 *
 * 优势:
 * - 使用Ceres::Jet自动计算雅可比矩阵，无需手动推导
 * - 模板化设计，支持任意状态/观测维度
 * - 分离predict/measure步骤，灵活组合
 * - 马氏距离门限检查，拒绝离群观测
 * - 自适应过程噪声，拒绝后宽松，接受后恢复
 */

#ifndef AIMER_COMMON_FILTER_ADAPTIVE_EKF_HPP
#define AIMER_COMMON_FILTER_ADAPTIVE_EKF_HPP

#include <algorithm>

#include <ceres/jet.h>

#include <Eigen/Dense>

namespace aimer::filter {

/**
 * @brief 门限检查更新结果
 */
enum class UpdateStatus {
    ACCEPTED,   // 观测被接受，正常更新
    REJECTED,   // 观测被拒绝（马氏距离过大）
    RESET       // 连续拒绝过多，已从观测重新初始化
};

/**
 * @brief 自动微分扩展卡尔曼滤波器
 *
 * @tparam N_X 状态向量维度
 * @tparam N_Y 观测向量维度
 *
 * 使用Ceres的Jet类型自动计算雅可比矩阵:
 * - predict: 状态转移函数 f(x) 的雅可比 F = ∂f/∂x
 * - measure: 观测函数 h(x) 的雅可比 H = ∂h/∂x
 */
template<int N_X, int N_Y>
class AdaptiveEkf {
public:
    // 类型别名
    using MatrixXX = Eigen::Matrix<double, N_X, N_X>;
    using MatrixYX = Eigen::Matrix<double, N_Y, N_X>;
    using MatrixXY = Eigen::Matrix<double, N_X, N_Y>;
    using MatrixYY = Eigen::Matrix<double, N_Y, N_Y>;
    using VectorX  = Eigen::Matrix<double, N_X, 1>;
    using VectorY  = Eigen::Matrix<double, N_Y, 1>;

    // 预测结果
    struct PredictResult {
        VectorX  x_p;    // 预测状态
        MatrixXX F;      // 状态转移雅可比
    };

    // 观测结果
    struct MeasureResult {
        VectorY  y_e;    // 预测观测
        MatrixYX H;      // 观测雅可比
    };

private:
    VectorX  x_;         // 状态估计
    MatrixXX P_;         // 状态协方差

    // 门限检查自适应状态
    int reject_count_ = 0;     // 连续拒绝次数
    double q_scale_ = 1.0;     // 过程噪声缩放因子

    static constexpr double INF = 1e9;

public:
    AdaptiveEkf() : x_(VectorX::Zero()), P_(MatrixXX::Identity() * INF) {}

    explicit AdaptiveEkf(const VectorX& x0) : x_(x0), P_(MatrixXX::Identity()) {}

    // ========================================================================
    // 初始化/访问
    // ========================================================================

    /**
     * @brief 初始化状态，协方差设为单位矩阵
     */
    void init(const VectorX& x0) {
        x_ = x0;
        P_ = MatrixXX::Identity();
        reset_gating_state();
    }

    /**
     * @brief 初始化状态和协方差
     */
    void init(const VectorX& x0, const MatrixXX& P0) {
        x_ = x0;
        P_ = P0;
        reset_gating_state();
    }

    /**
     * @brief 设置状态（不改变协方差）
     */
    void set_x(const VectorX& x) { x_ = x; }

    /**
     * @brief 获取当前状态
     */
    VectorX get_x() const { return x_; }

    /**
     * @brief 获取协方差矩阵
     */
    MatrixXX get_P() const { return P_; }

    /**
     * @brief 获取当前过程噪声缩放因子
     */
    double get_q_scale() const { return q_scale_; }

    /**
     * @brief 获取连续拒绝次数
     */
    int get_reject_count() const { return reject_count_; }

    /**
     * @brief 重置门限检查状态
     */
    void reset_gating_state() {
        reject_count_ = 0;
        q_scale_ = 1.0;
    }

    // ========================================================================
    // 预测步骤
    // ========================================================================

    /**
     * @brief 计算预测结果（不修改内部状态）
     *
     * @tparam PredictFunc 预测函数类型，签名: void(const T x_in[N_X], T x_out[N_X])
     * @param func 预测函数（支持Ceres::Jet自动微分）
     * @return PredictResult 包含预测状态和雅可比矩阵
     *
     * 示例:
     * @code
     * struct CVPredict {
     *     double dt;
     *     template<typename T>
     *     void operator()(const T x_in[6], T x_out[6]) const {
     *         x_out[0] = x_in[0] + dt * x_in[1];  // x += vx * dt
     *         x_out[1] = x_in[1];                  // vx 不变
     *         // ... 其他状态
     *     }
     * };
     * @endcode
     */
    template<typename PredictFunc>
    PredictResult predict(PredictFunc&& func) const {
        // 构造Jet变量，设置初值和单位导数
        ceres::Jet<double, N_X> x_jet[N_X];
        for (int i = 0; i < N_X; ++i) {
            x_jet[i].a = x_[i];       // 函数值 = 当前状态
            x_jet[i].v.setZero();
            x_jet[i].v[i] = 1.0;      // ∂x_i/∂x_i = 1
        }

        // 执行预测函数
        ceres::Jet<double, N_X> x_pred_jet[N_X];
        func(x_jet, x_pred_jet);

        // 提取预测状态和雅可比矩阵
        PredictResult result;
        for (int i = 0; i < N_X; ++i) {
            result.x_p[i] = x_pred_jet[i].a;
            result.F.row(i) = x_pred_jet[i].v.transpose();
        }

        return result;
    }

    /**
     * @brief 执行预测步骤（修改内部状态）
     *
     * @param func 预测函数
     * @param Q 过程噪声协方差矩阵
     */
    template<typename PredictFunc>
    void predict_forward(PredictFunc&& func, const MatrixXX& Q) {
        PredictResult res = predict(func);
        x_ = res.x_p;
        P_ = res.F * P_ * res.F.transpose() + Q;
    }

    /**
     * @brief 执行预测步骤，使用自适应缩放的过程噪声
     *
     * @param func 预测函数
     * @param Q_base 基础过程噪声协方差矩阵 (会乘以 q_scale_)
     */
    template<typename PredictFunc>
    void predict_forward_scaled(PredictFunc&& func, const MatrixXX& Q_base) {
        PredictResult res = predict(func);
        x_ = res.x_p;
        P_ = res.F * P_ * res.F.transpose() + Q_base * q_scale_;
    }

    // ========================================================================
    // 观测步骤
    // ========================================================================

    /**
     * @brief 计算观测结果（不修改内部状态）
     *
     * @tparam MeasureFunc 观测函数类型，签名: void(const T x[N_X], T y[N_Y])
     * @param func 观测函数（支持Ceres::Jet自动微分）
     * @return MeasureResult 包含预测观测和雅可比矩阵
     *
     * 示例（XYZ→YPD转换）:
     * @code
     * struct XyzToYpd {
     *     template<typename T>
     *     void operator()(const T x[6], T y[3]) const {
     *         T xyz[3] = {x[0], x[2], x[4]};
     *         y[0] = ceres::atan2(xyz[1], xyz[0]);  // yaw
     *         y[1] = ceres::atan2(xyz[2], ceres::sqrt(xyz[0]*xyz[0] + xyz[1]*xyz[1]));
     *         y[2] = ceres::sqrt(xyz[0]*xyz[0] + xyz[1]*xyz[1] + xyz[2]*xyz[2]);
     *     }
     * };
     * @endcode
     */
    template<typename MeasureFunc>
    MeasureResult measure(MeasureFunc&& func) const {
        // 构造Jet变量
        ceres::Jet<double, N_X> x_jet[N_X];
        for (int i = 0; i < N_X; ++i) {
            x_jet[i].a = x_[i];
            x_jet[i].v.setZero();
            x_jet[i].v[i] = 1.0;
        }

        // 执行观测函数
        ceres::Jet<double, N_X> y_jet[N_Y];
        func(x_jet, y_jet);

        // 提取预测观测和雅可比矩阵
        MeasureResult result;
        for (int i = 0; i < N_Y; ++i) {
            result.y_e[i] = y_jet[i].a;
            result.H.row(i) = y_jet[i].v.transpose();
        }

        return result;
    }

    /**
     * @brief 执行观测更新步骤（修改内部状态）
     *
     * @param func 观测函数
     * @param y 实际观测值
     * @param R 观测噪声协方差矩阵
     */
    template<typename MeasureFunc>
    void update_forward(MeasureFunc&& func, const VectorY& y, const MatrixYY& R) {
        MeasureResult res = measure(func);

        // 计算卡尔曼增益: K = P * H^T * (H * P * H^T + R)^{-1}
        MatrixYY S = res.H * P_ * res.H.transpose() + R;
        MatrixXY K = P_ * res.H.transpose() * S.inverse();

        // 更新状态: x = x + K * (y - y_pred)
        x_ = x_ + K * (y - res.y_e);

        // 更新协方差: P = (I - K * H) * P
        P_ = (MatrixXX::Identity() - K * res.H) * P_;
    }

    /**
     * @brief 带马氏距离门限检查的观测更新
     *
     * 马氏距离: d² = (y - ŷ)ᵀ S⁻¹ (y - ŷ)
     *
     * 策略:
     * - d² > threshold: 拒绝观测，增大 q_scale_
     * - 连续拒绝 >= max_reject: 从 reset_state 重新初始化
     * - 接受观测: 正常更新，q_scale_ 慢慢恢复
     *
     * @param func 观测函数
     * @param y 实际观测值
     * @param R 观测噪声协方差矩阵
     * @param reset_state 重置时使用的状态 (从当前观测构建)
     * @param chi2_threshold χ² 门限 (建议: 自由度=N_Y, p=0.01)
     * @param max_reject 最大连续拒绝次数
     * @param q_scale_increase 拒绝时 q_scale_ 乘数
     * @param q_scale_decay 接受时 q_scale_ 乘数 (min 1.0)
     * @return UpdateStatus 更新结果
     */
    template<typename MeasureFunc>
    UpdateStatus update_forward_gated(
        MeasureFunc&& func,
        const VectorY& y,
        const MatrixYY& R,
        const VectorX& reset_state,
        double chi2_threshold = 9.21,
        int max_reject = 5,
        double q_scale_increase = 1.5,
        double q_scale_decay = 0.9
    ) {
        MeasureResult res = measure(func);

        // 计算新息协方差: S = H P Hᵀ + R
        MatrixYY S = res.H * P_ * res.H.transpose() + R;

        // 计算新息 (观测残差)
        VectorY innovation = y - res.y_e;

        // 计算马氏距离平方: d² = innovationᵀ S⁻¹ innovation
        double mahalanobis_sq = innovation.transpose() * S.inverse() * innovation;

        // 门限检查 (宽松期门限也放大)
        double threshold = chi2_threshold;
        if (q_scale_ > 1.0) {
            threshold *= q_scale_;
        }

        if (mahalanobis_sq > threshold) {
            // ========== 拒绝观测 ==========
            reject_count_++;
            q_scale_ *= q_scale_increase;

            // 连续拒绝过多 → 模型可能完全错了，reset
            if (reject_count_ >= max_reject) {
                x_ = reset_state;
                P_ = MatrixXX::Identity();
                reject_count_ = 0;
                q_scale_ = 1.0;
                return UpdateStatus::RESET;
            }

            return UpdateStatus::REJECTED;
        }

        // ========== 接受观测 ==========
        reject_count_ = 0;
        q_scale_ = std::max(1.0, q_scale_ * q_scale_decay);

        // 标准 EKF 更新
        MatrixXY K = P_ * res.H.transpose() * S.inverse();
        x_ = x_ + K * innovation;
        P_ = (MatrixXX::Identity() - K * res.H) * P_;

        return UpdateStatus::ACCEPTED;
    }

    /**
     * @brief 获取上次更新的马氏距离 (调试用)
     *
     * 注意: 需要在 update_forward_gated 后立即调用才准确
     */
    template<typename MeasureFunc>
    double compute_mahalanobis_sq(MeasureFunc&& func, const VectorY& y, const MatrixYY& R) const {
        MeasureResult res = measure(func);
        MatrixYY S = res.H * P_ * res.H.transpose() + R;
        VectorY innovation = y - res.y_e;
        return innovation.transpose() * S.inverse() * innovation;
    }

    // ========================================================================
    // 组合操作
    // ========================================================================

    /**
     * @brief 一步完成预测+更新
     *
     * @param predict_func 预测函数
     * @param measure_func 观测函数
     * @param y 实际观测值
     * @param Q 过程噪声
     * @param R 观测噪声
     */
    template<typename PredictFunc, typename MeasureFunc>
    void update(
        PredictFunc&& predict_func,
        MeasureFunc&& measure_func,
        const VectorY& y,
        const MatrixXX& Q,
        const MatrixYY& R
    ) {
        predict_forward(predict_func, Q);
        update_forward(measure_func, y, R);
    }
};

}  // namespace aimer::filter

#endif  // AIMER_COMMON_FILTER_ADAPTIVE_EKF_HPP
