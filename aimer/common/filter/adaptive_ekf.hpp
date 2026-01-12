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
 */

#ifndef AIMER_COMMON_FILTER_ADAPTIVE_EKF_HPP
#define AIMER_COMMON_FILTER_ADAPTIVE_EKF_HPP

#include <ceres/jet.h>

#include <Eigen/Dense>

namespace aimer::filter {

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
    }

    /**
     * @brief 初始化状态和协方差
     */
    void init(const VectorX& x0, const MatrixXX& P0) {
        x_ = x0;
        P_ = P0;
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
