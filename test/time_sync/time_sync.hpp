//
// 相机-IMU 时间戳标定模块
// 通过最小化静止目标在世界坐标系下的位置方差来估计时间偏移
//
// 原理:
//   静止目标在世界坐标系下位置不变
//   p_world = R_imu(t_cam + delta_t) * R_gimbal2imu * R_cam2gimbal * p_cam
//   找到 delta_t 使得 p_world 的方差最小
//
// 使用方法:
//   1. 将相机对准静止目标（装甲板、墙角等）
//   2. 晃动云台采集数据（相机和IMU分别记录时间戳）
//   3. 调用 calibrate() 获取时间偏移
//

#ifndef TIME_SYNC_HPP
#define TIME_SYNC_HPP

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ceres/ceres.h>
#include <fmt/format.h>

namespace time_sync {

// ============================================================================
// 时间戳类型
// ============================================================================

using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

// 将 TimePoint 转换为 double (秒)
inline double to_seconds(TimePoint tp) {
    return std::chrono::duration<double>(tp.time_since_epoch()).count();
}

// 时间工具函数
constexpr double us_to_s(double us) { return us * 1e-6; }
constexpr double s_to_us(double s)  { return s * 1e6; }
constexpr double ms_to_s(double ms) { return ms * 1e-3; }
constexpr double s_to_ms(double s)  { return s * 1e3; }

// ============================================================================
// 数据结构
// ============================================================================

/**
 * @brief IMU数据点 - 带独立时间戳
 */
struct ImuSample {
    double timestamp;               // 接收时间戳 (秒, from steady_clock)
    Eigen::Quaterniond orientation; // 姿态四元数

    ImuSample() = default;
    ImuSample(double t, const Eigen::Quaterniond& q)
        : timestamp(t), orientation(q.normalized()) {}

    // 从 TimePoint 构造
    ImuSample(TimePoint tp, const Eigen::Quaterniond& q)
        : timestamp(to_seconds(tp)), orientation(q.normalized()) {}

    // 从欧拉角构造 (yaw, pitch, roll in degrees)
    static ImuSample from_euler_deg(TimePoint tp, double yaw, double pitch, double roll) {
        // 角度转弧度
        double y = yaw * M_PI / 180.0;
        double p = pitch * M_PI / 180.0;
        double r = roll * M_PI / 180.0;

        // ZYX顺序 (yaw-pitch-roll)
        Eigen::Quaterniond q = Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ())
                             * Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY())
                             * Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX());
        return ImuSample(tp, q);
    }
};

/**
 * @brief 相机观测点 - 带独立时间戳
 */
struct CamSample {
    double timestamp;           // 拍摄时间戳 (秒, from steady_clock)
    Eigen::Vector3d point_c;    // 相机坐标系下的3D点

    CamSample() = default;
    CamSample(double t, const Eigen::Vector3d& p)
        : timestamp(t), point_c(p) {}

    // 从 TimePoint 构造
    CamSample(TimePoint tp, const Eigen::Vector3d& p)
        : timestamp(to_seconds(tp)), point_c(p) {}

    CamSample(TimePoint tp, double x, double y, double z)
        : timestamp(to_seconds(tp)), point_c(x, y, z) {}
};

// ============================================================================
// IMU姿态插值器 (SLERP)
// ============================================================================

/**
 * @brief IMU姿态插值器
 * 使用球面线性插值(SLERP)在相邻四元数之间插值
 */
class ImuInterpolator {
public:
    ImuInterpolator() = default;

    explicit ImuInterpolator(std::vector<ImuSample> samples)
        : data_(std::move(samples)) {
        // 按时间戳排序
        std::sort(data_.begin(), data_.end(),
            [](const ImuSample& a, const ImuSample& b) {
                return a.timestamp < b.timestamp;
            });
    }

    /**
     * @brief 在指定时刻插值姿态
     * @param t 查询时刻（秒）
     * @return 插值后的四元数
     */
    [[nodiscard]]
    Eigen::Quaterniond operator()(double t) const {
        if (data_.empty()) {
            return Eigen::Quaterniond::Identity();
        }

        // 边界处理
        if (t <= data_.front().timestamp) {
            return data_.front().orientation;
        }
        if (t >= data_.back().timestamp) {
            return data_.back().orientation;
        }

        // 二分查找
        auto it = std::lower_bound(data_.begin(), data_.end(), t,
            [](const ImuSample& s, double time) {
                return s.timestamp < time;
            });

        size_t idx = std::distance(data_.begin(), it);
        if (idx == 0) idx = 1;

        const auto& d0 = data_[idx - 1];
        const auto& d1 = data_[idx];

        // 插值系数
        double alpha = (t - d0.timestamp) / (d1.timestamp - d0.timestamp);
        alpha = std::clamp(alpha, 0.0, 1.0);

        // SLERP
        return d0.orientation.slerp(alpha, d1.orientation);
    }

    [[nodiscard]] double min_time() const {
        return data_.empty() ? 0.0 : data_.front().timestamp;
    }

    [[nodiscard]] double max_time() const {
        return data_.empty() ? 0.0 : data_.back().timestamp;
    }

    [[nodiscard]] size_t size() const { return data_.size(); }
    [[nodiscard]] bool empty() const { return data_.empty(); }

private:
    std::vector<ImuSample> data_;
};

// ============================================================================
// 静态外参 (用于 Camera → Gimbal → World 变换)
// ============================================================================

/**
 * @brief 外参配置
 * 从 camera.yaml 加载的静态参数
 */
struct ExtrinsicParams {
    Eigen::Matrix3d R_cam2gimbal = Eigen::Matrix3d::Identity();  // 相机到云台
    Eigen::Matrix3d R_gimbal2imu = Eigen::Matrix3d::Identity();  // 云台到IMU

    // 组合旋转: Camera → Gimbal → IMU (不含动态的 IMU → World)
    [[nodiscard]]
    Eigen::Matrix3d R_cam2imu() const {
        return R_gimbal2imu * R_cam2gimbal;
    }
};

// ============================================================================
// 方差计算
// ============================================================================

/**
 * @brief 计算给定 delta_t 下转换到世界系后的方差
 *
 * 变换链: p_world = R_imu(t_cam + delta_t) * R_gimbal2imu * R_cam2gimbal * p_cam
 *                 = R_imu(t_cam + delta_t) * R_cam2imu * p_cam
 *
 * 静止目标的 p_world 应为常数，方差最小时 delta_t 最优
 */
inline double compute_variance(
    const std::vector<CamSample>& cam_samples,
    const ImuInterpolator& imu_interp,
    const ExtrinsicParams& extrinsic,
    double delta_t
) {
    if (cam_samples.size() < 2) {
        return std::numeric_limits<double>::infinity();
    }

    // 预计算静态旋转
    Eigen::Matrix3d R_cam2imu = extrinsic.R_cam2imu();

    // 转换所有点到世界坐标系
    std::vector<Eigen::Vector3d> world_points;
    world_points.reserve(cam_samples.size());

    for (const auto& cs : cam_samples) {
        double t_imu = cs.timestamp + delta_t;
        Eigen::Quaterniond q_imu = imu_interp(t_imu);

        // p_world = R_imu * R_cam2imu * p_cam
        Eigen::Vector3d p_world = q_imu.toRotationMatrix() * R_cam2imu * cs.point_c;
        world_points.push_back(p_world);
    }

    // 计算均值
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto& p : world_points) {
        mean += p;
    }
    mean /= static_cast<double>(world_points.size());

    // 计算方差
    double variance = 0.0;
    for (const auto& p : world_points) {
        variance += (p - mean).squaredNorm();
    }
    return variance / static_cast<double>(world_points.size());
}

inline double compute_std(
    const std::vector<CamSample>& cam_samples,
    const ImuInterpolator& imu_interp,
    const ExtrinsicParams& extrinsic,
    double delta_t
) {
    return std::sqrt(compute_variance(cam_samples, imu_interp, extrinsic, delta_t));
}

// ============================================================================
// Ceres优化
// ============================================================================

namespace detail {

// 数值微分代价函数（SLERP不便于自动微分）
class TimeSyncCostFunction : public ceres::CostFunction {
public:
    TimeSyncCostFunction(
        const std::vector<CamSample>& cam_samples,
        const ImuInterpolator& imu_interp,
        const ExtrinsicParams& extrinsic
    ) : cam_samples_(cam_samples)
      , imu_interp_(imu_interp)
      , extrinsic_(extrinsic) {
        set_num_residuals(1);
        mutable_parameter_block_sizes()->push_back(1);
    }

    bool Evaluate(
        double const* const* parameters,
        double* residuals,
        double** jacobians
    ) const override {
        double delta_t = parameters[0][0];
        double var = compute_variance(cam_samples_, imu_interp_, extrinsic_, delta_t);

        residuals[0] = std::sqrt(var);

        if (jacobians && jacobians[0]) {
            constexpr double eps = 1e-6;
            double var_plus = compute_variance(cam_samples_, imu_interp_, extrinsic_, delta_t + eps);
            double var_minus = compute_variance(cam_samples_, imu_interp_, extrinsic_, delta_t - eps);
            double d_var = (var_plus - var_minus) / (2.0 * eps);

            if (var > 1e-12) {
                jacobians[0][0] = d_var / (2.0 * std::sqrt(var));
            } else {
                jacobians[0][0] = 0.0;
            }
        }
        return true;
    }

private:
    const std::vector<CamSample>& cam_samples_;
    const ImuInterpolator& imu_interp_;
    const ExtrinsicParams& extrinsic_;
};

}  // namespace detail

// ============================================================================
// 标定结果
// ============================================================================

struct CalibrationResult {
    double delta_t_us = 0.0;    // 时间偏移（微秒）
    double delta_t_s = 0.0;     // 时间偏移（秒）
    double initial_std = 0.0;   // 优化前标准差（米）
    double final_std = 0.0;     // 优化后标准差（米）
    bool success = false;
    std::string message;

    void print() const {
        fmt::print("==================================================\n");
        fmt::print("相机-IMU 时间戳标定结果\n");
        fmt::print("==================================================\n");
        fmt::print("状态: {}\n", success ? "成功" : "失败");
        fmt::print("信息: {}\n", message);
        fmt::print("时间偏移: {:.1f} us ({:.3f} ms)\n", delta_t_us, delta_t_us / 1000.0);
        fmt::print("优化前标准差: {:.3f} mm\n", initial_std * 1000.0);
        fmt::print("优化后标准差: {:.3f} mm\n", final_std * 1000.0);
        fmt::print("==================================================\n");
    }
};

// ============================================================================
// 主标定函数
// ============================================================================

/**
 * @brief 标定相机-IMU时间偏移
 *
 * @param cam_samples 相机观测点序列（静止目标）- 带独立时间戳
 * @param imu_samples IMU数据序列 - 带独立时间戳
 * @param extrinsic 外参配置
 * @param search_range_ms 粗搜索范围（毫秒），默认±100ms
 * @param search_step_ms 粗搜索步长（毫秒），默认1ms
 * @param verbose 是否输出详细信息
 * @return CalibrationResult
 */
inline CalibrationResult calibrate(
    const std::vector<CamSample>& cam_samples,
    const std::vector<ImuSample>& imu_samples,
    const ExtrinsicParams& extrinsic = ExtrinsicParams(),
    double search_range_ms = 100.0,
    double search_step_ms = 1.0,
    bool verbose = true
) {
    CalibrationResult result;

    // 数据检查
    if (cam_samples.size() < 10) {
        result.message = fmt::format("相机数据点不足: {} < 10", cam_samples.size());
        return result;
    }
    if (imu_samples.size() < 10) {
        result.message = fmt::format("IMU数据点不足: {} < 10", imu_samples.size());
        return result;
    }

    ImuInterpolator imu_interp(imu_samples);

    // 时间范围
    double cam_min = cam_samples.front().timestamp;
    double cam_max = cam_samples.back().timestamp;
    for (const auto& cs : cam_samples) {
        cam_min = std::min(cam_min, cs.timestamp);
        cam_max = std::max(cam_max, cs.timestamp);
    }

    if (verbose) {
        fmt::print("数据统计:\n");
        fmt::print("  相机帧数: {}\n", cam_samples.size());
        fmt::print("  IMU帧数: {}\n", imu_samples.size());
        fmt::print("  相机时间跨度: {:.3f}s\n", cam_max - cam_min);
        fmt::print("  IMU时间跨度: {:.3f}s\n", imu_interp.max_time() - imu_interp.min_time());

        // 检查时间重叠
        double overlap_start = std::max(cam_min, imu_interp.min_time());
        double overlap_end = std::min(cam_max, imu_interp.max_time());
        if (overlap_end > overlap_start) {
            fmt::print("  时间重叠: {:.3f}s\n", overlap_end - overlap_start);
        } else {
            fmt::print("  警告: 相机和IMU时间无重叠!\n");
        }
    }

    result.initial_std = compute_std(cam_samples, imu_interp, extrinsic, 0.0);

    // 粗搜索
    double search_range = search_range_ms / 1000.0;
    double search_step = search_step_ms / 1000.0;

    double best_delta_t = 0.0;
    double min_variance = std::numeric_limits<double>::max();

    for (double dt = -search_range; dt <= search_range; dt += search_step) {
        double var = compute_variance(cam_samples, imu_interp, extrinsic, dt);
        if (var < min_variance) {
            min_variance = var;
            best_delta_t = dt;
        }
    }

    if (verbose) {
        fmt::print("\n粗搜索结果: delta_t = {:.1f} us, std = {:.3f} mm\n",
            best_delta_t * 1e6, std::sqrt(min_variance) * 1000.0);
    }

    // Ceres精细优化
    double delta_t = best_delta_t;

    ceres::Problem problem;
    problem.AddResidualBlock(
        new detail::TimeSyncCostFunction(cam_samples, imu_interp, extrinsic),
        nullptr,
        &delta_t
    );

    problem.SetParameterLowerBound(&delta_t, 0, -search_range);
    problem.SetParameterUpperBound(&delta_t, 0, search_range);

    ceres::Solver::Options options;
    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = verbose;
    options.function_tolerance = 1e-10;
    options.parameter_tolerance = 1e-10;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (verbose) {
        fmt::print("\n{}\n", summary.BriefReport());
    }

    result.delta_t_s = delta_t;
    result.delta_t_us = delta_t * 1e6;
    result.final_std = compute_std(cam_samples, imu_interp, extrinsic, delta_t);
    result.success = summary.IsSolutionUsable();
    result.message = summary.IsSolutionUsable()
        ? "标定成功"
        : fmt::format("优化失败: {}", summary.message);

    return result;
}

}  // namespace time_sync

#endif  // TIME_SYNC_HPP
