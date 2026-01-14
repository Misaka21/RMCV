//
// Created by nuc11 on 2025/10/5.
// 数学工具库
//

#ifndef RMCV_MATH_HPP
#define RMCV_MATH_HPP

#include <Eigen/Dense>
#include <cmath>
#include <opencv2/core/types.hpp>

namespace aimer::math {

// ============================================================================
// 1. 球坐标结构体
// ============================================================================

struct YpdCoord {
    double yaw   = 0.;  // 偏航角 (rad)
    double pitch = 0.;  // 俯仰角 (rad)
    double dis   = 0.;  // 距离 (m)

    YpdCoord() = default;
    YpdCoord(double y, double p, double d) noexcept : yaw(y), pitch(p), dis(d) {}

    // 转为Eigen向量
    Eigen::Vector3d to_vec() const noexcept { return {yaw, pitch, dis}; }

    // 运算符重载
    YpdCoord operator+(const YpdCoord& other) const noexcept {
        return {yaw + other.yaw, pitch + other.pitch, dis + other.dis};
    }

    YpdCoord& operator+=(const YpdCoord& other) noexcept {
        yaw += other.yaw;
        pitch += other.pitch;
        dis += other.dis;
        return *this;
    }
};

// ============================================================================
// 2. 通用数学函数
// ============================================================================

// 平方
template<typename T>
inline T sq(const T& x) noexcept { return x * x; }

// 模长
inline double norm(double x, double y) noexcept {
    return std::sqrt(x * x + y * y);
}

// 比例 (小/大)
inline double get_ratio(double x, double y) noexcept {
    if (x == 0. || y == 0.) return 0.;
    return (x < y) ? x / y : y / x;
}

// sigmoid
inline double sigmoid(double x) noexcept {
    return 1.0 / (1.0 + std::exp(-x));
}

// clamp with default
template<typename T>
inline T clamp_default(const T& x, const T& lower, const T& upper, const T& default_val) noexcept {
    if (x < lower || x > upper) return default_val;
    return x;
}

// 判断nan或inf
inline bool is_nan_or_inf(double x) noexcept {
    return std::isnan(x) || std::isinf(x);
}

// ============================================================================
// 3. 角度工具
// ============================================================================

// 度 → 弧度
constexpr double deg2rad(double deg) noexcept { return deg * M_PI / 180.0; }

// 弧度 → 度
constexpr double rad2deg(double rad) noexcept { return rad * 180.0 / M_PI; }

// 限制到 (-π, π]
inline double reduced_angle(double x) noexcept {
    return std::atan2(std::sin(x), std::cos(x));
}

// 别名，与其他模块兼容
inline double normalize_angle(double x) noexcept {
    return reduced_angle(x);
}

// 两角度之差，结果在 (-π, π]
inline double angle_diff(double a1, double a2) noexcept {
    return reduced_angle(a2 - a1);
}

// 获取最近的等价角度 (处理±π跨越)
// 用于EKF更新时保持角度连续性
inline double get_closest_angle(double target, double current) noexcept {
    double diff = target - current;
    while (diff > M_PI)  diff -= 2 * M_PI;
    while (diff < -M_PI) diff += 2 * M_PI;
    return current + diff;
}

// 两向量夹角 [0, π]
inline double get_abs_angle(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2) noexcept {
    if (v1.norm() == 0. || v2.norm() == 0.) return 0.;
    return std::acos(v1.dot(v2) / (v1.norm() * v2.norm()));
}

// 向量角度
inline double get_theta(const Eigen::Vector2d& v) noexcept {
    return std::atan2(v.y(), v.x());
}

// 四元数 → (yaw, pitch)
inline std::pair<double, double> quat_to_yaw_pitch(const Eigen::Quaterniond& q) noexcept {
    double yaw = std::atan2(2*(q.w()*q.z() + q.x()*q.y()),
                           1 - 2*(q.y()*q.y() + q.z()*q.z()));
    double pitch = std::asin(2*(q.w()*q.y() - q.z()*q.x()));
    return {yaw, pitch};
}

// ============================================================================
// 4. 坐标转换 (xyz ↔ ypd)
// ============================================================================

// 直角坐标 → 球坐标 (x前, y左, z上) - ROS惯例
// yaw: 从x轴(前)向y轴(左)的角度，左为正
// pitch: 从水平面向z轴(上)的角度，上为正
inline YpdCoord xyz_to_ypd(const Eigen::Vector3d& xyz) noexcept {
    double x = xyz.x(), y = xyz.y(), z = xyz.z();
    double dis_xy = std::sqrt(x * x + y * y);
    return {
        std::atan2(y, x),       // yaw: 左为正
        std::atan2(z, dis_xy),  // pitch: 上为正
        xyz.norm()              // distance
    };
}

// 球坐标 → 直角坐标 (x前, y左, z上)
inline Eigen::Vector3d ypd_to_xyz(const YpdCoord& ypd) noexcept {
    double cos_pitch = std::cos(ypd.pitch);
    return {
        ypd.dis * cos_pitch * std::cos(ypd.yaw),  // x = 前
        ypd.dis * cos_pitch * std::sin(ypd.yaw),  // y = 左
        ypd.dis * std::sin(ypd.pitch)             // z = 上
    };
}

// Vector3d 重载
inline Eigen::Vector3d ypd_to_xyz(const Eigen::Vector3d& ypd) noexcept {
    double yaw = ypd.x(), pitch = ypd.y(), dis = ypd.z();
    double cos_pitch = std::cos(pitch);
    return {
        dis * cos_pitch * std::cos(yaw),  // x = 前
        dis * cos_pitch * std::sin(yaw),  // y = 左
        dis * std::sin(pitch)             // z = 上
    };
}

// 相机坐标系 xyz (z前, x右, y下) → ypd
inline YpdCoord camera_xyz_to_ypd(const Eigen::Vector3d& xyz) noexcept {
    return {
        std::atan2(xyz.x(), xyz.z()),  // yaw: 右为正
        std::atan2(xyz.y(), xyz.z()),  // pitch: 下为正
        xyz.norm()
    };
}

// 相机坐标系 ypd → xyz
inline Eigen::Vector3d camera_ypd_to_xyz(const YpdCoord& ypd) noexcept {
    double t1 = std::tan(ypd.yaw);
    double t2 = std::tan(ypd.pitch);
    double z = std::sqrt(ypd.dis * ypd.dis / (t1 * t1 + t2 * t2 + 1.));
    return {z * t1, z * t2, z};
}

// ============================================================================
// 5. 几何计算
// ============================================================================

// 两点距离
inline float get_dis(const cv::Point2f& p1, const cv::Point2f& p2) noexcept {
    return std::sqrt(sq(p1.x - p2.x) + sq(p1.y - p2.y));
}

// 叉积 (p1为原点)
inline float get_cross(const cv::Point2f& p1, const cv::Point2f& p2, const cv::Point2f& p3) noexcept {
    return (p2.x - p1.x) * (p3.y - p1.y) - (p3.x - p1.x) * (p2.y - p1.y);
}

// 四边形面积
inline float get_area(const cv::Point2f pts[4]) noexcept {
    return std::fabs(get_cross(pts[0], pts[1], pts[2]))
         + std::fabs(get_cross(pts[0], pts[2], pts[3]));
}

// 四边形面积 (vector 版本)
inline float get_area(const std::vector<cv::Point2f>& pts) noexcept {
    if (pts.size() < 4) return 0;
    return std::fabs(get_cross(pts[0], pts[1], pts[2]))
         + std::fabs(get_cross(pts[0], pts[2], pts[3]));
}

// 2D向量旋转
inline Eigen::Vector2d rotate(const Eigen::Vector2d& v, double angle) noexcept {
    double c = std::cos(angle), s = std::sin(angle);
    return {c * v.x() - s * v.y(), s * v.x() + c * v.y()};
}

// ============================================================================
// 6. reduce 系列 (变参模板)
// ============================================================================

template<class F, class T, class... Ts>
T reduce(F&& func, T x, Ts... xs) {
    if constexpr (sizeof...(Ts) > 0) {
        return func(x, reduce(std::forward<F>(func), xs...));
    } else {
        return x;
    }
}

template<class T, class... Ts>
T reduce_min(T x, Ts... xs) {
    return reduce([](auto a, auto b) { return std::min(a, b); }, x, xs...);
}

template<class T, class... Ts>
T reduce_max(T x, Ts... xs) {
    return reduce([](auto a, auto b) { return std::max(a, b); }, x, xs...);
}

// ============================================================================
// 7. 三分搜索 (黄金分割法)
// ============================================================================

/**
 * @brief 三分搜索器 (黄金分割法)
 *
 * 用于在单峰函数上寻找最小值点
 * 适用于: 三分求 yaw, 重投影代价最小化等
 */
class Trisection {
public:
    /**
     * @brief 在 [left, right] 区间内寻找使 cost_function 最小的点
     * @tparam ValueT 数值类型 (通常是 double)
     * @tparam Func 代价函数类型，签名: ValueT(ValueT)
     * @param left 搜索区间左端点
     * @param right 搜索区间右端点
     * @param cost_function 代价函数
     * @param iterations 迭代次数 (12次约0.5度精度)
     * @return {最优值, 区间宽度}
     */
    template<typename ValueT, class Func>
    std::pair<ValueT, ValueT> find(
        ValueT left,
        ValueT right,
        Func&& cost_function,
        int iterations = 12
    ) {
        // 黄金分割比
        constexpr ValueT phi = 0.6180339887498949;  // (sqrt(5) - 1) / 2

        ValueT ml_cost = 0, mr_cost = 0;
        int reserved = -1;  // 缓存哪个点的代价 (0=ml, 1=mr)

        for (int i = 0; i < iterations; ++i) {
            ValueT ml = left + (right - left) * (1.0 - phi);
            ValueT mr = left + (right - left) * phi;

            // 只计算未缓存的点
            if (reserved != 0) {
                ml_cost = cost_function(ml);
            }
            if (reserved != 1) {
                mr_cost = cost_function(mr);
            }

            // 保留代价更小的一侧
            if (ml_cost < mr_cost) {
                right = mr;
                mr_cost = ml_cost;
                reserved = 1;
            } else {
                left = ml;
                ml_cost = mr_cost;
                reserved = 0;
            }
        }

        return {(left + right) / ValueT(2), right - left};
    }
};

} // namespace aimer::math

#endif //RMCV_MATH_HPP
