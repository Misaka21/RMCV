/**
 * @file single_filter.hpp
 * @brief 一维卡尔曼滤波器 (基于 AdaptiveEkf)
 *
 * 用于平滑标量观测值，如延迟估计
 */

#ifndef AIMER_COMMON_FILTER_SINGLE_FILTER_HPP
#define AIMER_COMMON_FILTER_SINGLE_FILTER_HPP

#include "adaptive_ekf.hpp"

namespace aimer::filter {

/**
 * @brief 多项式预测模型
 *
 * 状态向量: [x, dx/dt, d²x/dt², ...]
 * 泰勒展开预测: x(t+dt) = x + dx*dt + d²x*dt²/2 + ...
 */
template<int ORDER>
struct PolynomialPredict {
    double dt;

    explicit PolynomialPredict(double dt_) : dt(dt_) {}

    template<typename T>
    void operator()(const T x_in[ORDER], T x_out[ORDER]) const {
        // 计算泰勒系数: 1, dt, dt²/2!, dt³/3!, ...
        double coeff[ORDER];
        coeff[0] = 1.0;
        for (int i = 1; i < ORDER; ++i) {
            coeff[i] = coeff[i - 1] * dt / static_cast<double>(i);
        }

        // 泰勒展开
        for (int i = 0; i < ORDER; ++i) {
            x_out[i] = T(0);
            for (int j = i; j < ORDER; ++j) {
                x_out[i] += coeff[j - i] * x_in[j];
            }
        }
    }
};

/**
 * @brief 直接观测模型 (只观测第一个状态)
 */
template<int ORDER>
struct DirectMeasure {
    template<typename T>
    void operator()(const T x[ORDER], T y[1]) const {
        y[0] = x[0];
    }
};

/**
 * @brief 一维滤波器
 *
 * @tparam ORDER 阶数 (1=常数, 2=匀速, 3=匀加速)
 *
 * 用法:
 * @code
 * SingleFilter<2> filter;  // 二阶 (匀速模型)
 * filter.update(measurement, current_time, {1.0, 10.0}, {100.0});
 * double predicted = filter.predict(future_time)[0];
 * @endcode
 */
template<int ORDER>
class SingleFilter {
public:
    using Ekf = AdaptiveEkf<ORDER, 1>;
    using VectorX = Eigen::Matrix<double, ORDER, 1>;
    using MatrixXX = Eigen::Matrix<double, ORDER, ORDER>;
    using MatrixYY = Eigen::Matrix<double, 1, 1>;

    SingleFilter() = default;

    /**
     * @brief 初始化状态
     */
    void init(const VectorX& x0) {
        ekf_.init(x0);
        t_ = 0;
    }

    /**
     * @brief 设置状态
     */
    void set_x(const VectorX& x) {
        ekf_.set_x(x);
    }

    /**
     * @brief 设置时间
     */
    void set_t(double t) {
        t_ = t;
    }

    /**
     * @brief 获取当前状态
     */
    VectorX get_x() const {
        return ekf_.get_x();
    }

    /**
     * @brief 预测到指定时刻的状态
     */
    VectorX predict(double t) const {
        PolynomialPredict<ORDER> predict_func(t - t_);
        auto result = ekf_.predict(predict_func);
        return result.x_p;
    }

    /**
     * @brief 更新滤波器
     *
     * @param measurement 观测值
     * @param t 观测时间
     * @param q_diag Q矩阵对角线元素
     * @param r R矩阵 (1x1)
     */
    void update(
        double measurement,
        double t,
        const std::array<double, ORDER>& q_diag,
        double r
    ) {
        // 构造 Q 矩阵 (对角)
        MatrixXX Q = MatrixXX::Zero();
        for (int i = 0; i < ORDER; ++i) {
            Q(i, i) = q_diag[i];
        }

        // 构造 R 矩阵
        MatrixYY R;
        R << r;

        // 构造观测向量
        Eigen::Matrix<double, 1, 1> y;
        y << measurement;

        // 预测+更新
        PolynomialPredict<ORDER> predict_func(t - t_);
        DirectMeasure<ORDER> measure_func;
        ekf_.update(predict_func, measure_func, y, Q, R);

        t_ = t;
    }

    /**
     * @brief 简化的更新接口 (使用 vector)
     */
    void update(
        double measurement,
        double t,
        const std::vector<double>& q_vec,
        const std::vector<double>& r_vec
    ) {
        std::array<double, ORDER> q_diag{};
        for (int i = 0; i < ORDER && i < static_cast<int>(q_vec.size()); ++i) {
            q_diag[i] = q_vec[i];
        }
        double r = r_vec.empty() ? 1.0 : r_vec[0];
        update(measurement, t, q_diag, r);
    }

private:
    Ekf ekf_;
    double t_ = 0;
};

}  // namespace aimer::filter

#endif  // AIMER_COMMON_FILTER_SINGLE_FILTER_HPP
