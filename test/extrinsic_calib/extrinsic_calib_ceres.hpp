//
// Ceres 非线性优化器
// 自动微分求解最优外参
//

#ifndef RMCV_EXTRINSIC_CALIB_CERES_HPP
#define RMCV_EXTRINSIC_CALIB_CERES_HPP

#include "extrinsic_calib.hpp"

#include <ceres/ceres.h>
#include <ceres/rotation.h>

namespace extrinsic_calib {

// ============================================================================
// Ceres 残差函数
// ============================================================================

// 每个观测点的残差: World坐标与均值的差
struct ExtrinsicResidual {
    ExtrinsicResidual(
        const Eigen::Vector3d& p_camera,
        const Eigen::Quaterniond& q_imu,
        const Eigen::Matrix3d& R_gimbal2imu,
        const Eigen::Matrix3d& R_cam2gimbal_base)
        : p_camera_(p_camera)
        , q_imu_(q_imu)
        , R_gimbal2imu_(R_gimbal2imu)
        , R_cam2gimbal_base_(R_cam2gimbal_base)
    {}

    template <typename T>
    bool operator()(
        const T* const offset,      // [ox, oy, oz]
        const T* const delta_rpy,   // [roll, pitch, yaw]
        const T* const world_pos,   // [wx, wy, wz] 标定板World位置 (共同参数)
        T* residual) const
    {
        // 1. 构建delta旋转 (ZYX欧拉角)
        T roll = delta_rpy[0];
        T pitch = delta_rpy[1];
        T yaw = delta_rpy[2];

        // 手动计算ZYX旋转矩阵
        T cr = ceres::cos(roll), sr = ceres::sin(roll);
        T cp = ceres::cos(pitch), sp = ceres::sin(pitch);
        T cy = ceres::cos(yaw), sy = ceres::sin(yaw);

        // R = Rz(yaw) * Ry(pitch) * Rx(roll)
        T R_delta[9];
        R_delta[0] = cy * cp;
        R_delta[1] = cy * sp * sr - sy * cr;
        R_delta[2] = cy * sp * cr + sy * sr;
        R_delta[3] = sy * cp;
        R_delta[4] = sy * sp * sr + cy * cr;
        R_delta[5] = sy * sp * cr - cy * sr;
        R_delta[6] = -sp;
        R_delta[7] = cp * sr;
        R_delta[8] = cp * cr;

        // 2. Camera → Gimbal
        // R_cam2gimbal = R_base * R_delta
        T R_cam2gimbal[9];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                R_cam2gimbal[i * 3 + j] = T(0);
                for (int k = 0; k < 3; ++k) {
                    R_cam2gimbal[i * 3 + j] += T(R_cam2gimbal_base_(i, k)) * R_delta[k * 3 + j];
                }
            }
        }

        // 应用旋转和平移
        T p_gimbal[3];
        for (int i = 0; i < 3; ++i) {
            p_gimbal[i] = offset[i];
            for (int j = 0; j < 3; ++j) {
                p_gimbal[i] += R_cam2gimbal[i * 3 + j] * T(p_camera_[j]);
            }
        }

        // 3. Gimbal → Imu
        T p_imu[3];
        for (int i = 0; i < 3; ++i) {
            p_imu[i] = T(0);
            for (int j = 0; j < 3; ++j) {
                p_imu[i] += T(R_gimbal2imu_(i, j)) * p_gimbal[j];
            }
        }

        // 4. Imu → World
        Eigen::Matrix3d R_imu2world = q_imu_.toRotationMatrix();
        T p_world[3];
        for (int i = 0; i < 3; ++i) {
            p_world[i] = T(0);
            for (int j = 0; j < 3; ++j) {
                p_world[i] += T(R_imu2world(i, j)) * p_imu[j];
            }
        }

        // 5. 残差: 与共同标定板位置的差
        residual[0] = p_world[0] - world_pos[0];
        residual[1] = p_world[1] - world_pos[1];
        residual[2] = p_world[2] - world_pos[2];

        return true;
    }

    static ceres::CostFunction* Create(
        const Eigen::Vector3d& p_camera,
        const Eigen::Quaterniond& q_imu,
        const Eigen::Matrix3d& R_gimbal2imu,
        const Eigen::Matrix3d& R_cam2gimbal_base)
    {
        return new ceres::AutoDiffCostFunction<ExtrinsicResidual, 3, 3, 3, 3>(
            new ExtrinsicResidual(p_camera, q_imu, R_gimbal2imu, R_cam2gimbal_base));
    }

private:
    Eigen::Vector3d p_camera_;
    Eigen::Quaterniond q_imu_;
    Eigen::Matrix3d R_gimbal2imu_;
    Eigen::Matrix3d R_cam2gimbal_base_;
};

// ============================================================================
// Ceres 优化入口
// ============================================================================

inline CalibResult ceres_optimize(
    const std::vector<CalibSample>& samples,
    const BaseExtrinsic& base,
    const ExtrinsicParams* initial = nullptr,
    bool verbose = true)
{
    if (samples.size() < 3) {
        if (verbose) {
            fmt::print(fmt::fg(fmt::color::red), "错误: 至少需要3个样本\n");
        }
        return CalibResult{};
    }

    if (verbose) {
        fmt::print("\n========== Ceres 优化 ==========\n");
        fmt::print("样本数: {}\n\n", samples.size());
    }

    auto start_time = std::chrono::steady_clock::now();

    // 优化变量
    double offset[3] = {0, 0, 0};
    double delta_rpy[3] = {0, 0, 0};
    double world_pos[3] = {0, 0, 0};

    // 如果提供初始值，使用它
    if (initial) {
        offset[0] = initial->offset_x;
        offset[1] = initial->offset_y;
        offset[2] = initial->offset_z;
        delta_rpy[0] = initial->delta_roll;
        delta_rpy[1] = initial->delta_pitch;
        delta_rpy[2] = initial->delta_yaw;
    }

    // 用第一个样本估计初始world_pos
    ExtrinsicParams init_params;
    if (initial) init_params = *initial;
    Eigen::Vector3d init_world = camera_to_world(samples[0].p_camera, samples[0].q_imu, base, init_params);
    world_pos[0] = init_world.x();
    world_pos[1] = init_world.y();
    world_pos[2] = init_world.z();

    // 计算初始标准差
    double initial_std = compute_std(samples, base, init_params);

    // 构建问题
    ceres::Problem problem;

    for (const auto& sample : samples) {
        ceres::CostFunction* cost_function = ExtrinsicResidual::Create(
            sample.p_camera,
            sample.q_imu,
            base.R_gimbal2imu,
            base.R_cam2gimbal
        );

        problem.AddResidualBlock(cost_function, nullptr, offset, delta_rpy, world_pos);
    }

    // 设置参数边界
    problem.SetParameterLowerBound(offset, 0, -0.10);  // ±10cm
    problem.SetParameterUpperBound(offset, 0, 0.10);
    problem.SetParameterLowerBound(offset, 1, -0.10);
    problem.SetParameterUpperBound(offset, 1, 0.10);
    problem.SetParameterLowerBound(offset, 2, -0.10);
    problem.SetParameterUpperBound(offset, 2, 0.10);

    problem.SetParameterLowerBound(delta_rpy, 0, -0.35);  // ±20°
    problem.SetParameterUpperBound(delta_rpy, 0, 0.35);
    problem.SetParameterLowerBound(delta_rpy, 1, -0.35);
    problem.SetParameterUpperBound(delta_rpy, 1, 0.35);
    problem.SetParameterLowerBound(delta_rpy, 2, -0.35);
    problem.SetParameterUpperBound(delta_rpy, 2, 0.35);

    // 求解器选项
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = verbose;
    options.max_num_iterations = 200;
    options.function_tolerance = 1e-8;
    options.parameter_tolerance = 1e-8;

    // 求解
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    auto end_time = std::chrono::steady_clock::now();
    double solve_time = std::chrono::duration<double, std::milli>(end_time - start_time).count();

    if (verbose) {
        fmt::print("\n{}\n", summary.BriefReport());
    }

    // 构建结果
    CalibResult result;
    result.success = (summary.termination_type == ceres::CONVERGENCE);
    result.params.offset_x = offset[0];
    result.params.offset_y = offset[1];
    result.params.offset_z = offset[2];
    result.params.delta_roll = delta_rpy[0];
    result.params.delta_pitch = delta_rpy[1];
    result.params.delta_yaw = delta_rpy[2];
    result.initial_std = initial_std;
    result.final_std = compute_std(samples, base, result.params, &result.world_center);
    result.iterations = static_cast<int>(summary.iterations.size());
    result.solve_time_ms = solve_time;
    result.R_cam2gimbal_base = base.R_cam2gimbal;  // 保存基础旋转矩阵

    return result;
}

// ============================================================================
// 组合方法: 网格搜索初始化 + Ceres精修
// ============================================================================

inline CalibResult calibrate(
    const std::vector<CalibSample>& samples,
    const BaseExtrinsic& base,
    bool verbose = true)
{
    if (verbose) {
        fmt::print("\n============================================\n");
        fmt::print("      外参自动标定 (网格搜索 + Ceres)\n");
        fmt::print("============================================\n\n");
    }

    // 阶段1: 两阶段网格搜索获取初始值
    auto grid_result = two_stage_search(samples, base, verbose);

    if (verbose) {
        fmt::print("\n网格搜索完成:\n");
        grid_result.params.print();
        fmt::print("std = {:.4f}m\n\n", grid_result.final_std);
    }

    // 阶段2: Ceres精修
    auto ceres_result = ceres_optimize(samples, base, &grid_result.params, verbose);

    if (verbose) {
        fmt::print("\nCeres优化完成:\n");
    }

    // 合并统计
    ceres_result.initial_std = grid_result.initial_std;
    ceres_result.iterations += grid_result.iterations;
    ceres_result.solve_time_ms += grid_result.solve_time_ms;

    return ceres_result;
}

} // namespace extrinsic_calib

#endif // RMCV_EXTRINSIC_CALIB_CERES_HPP
