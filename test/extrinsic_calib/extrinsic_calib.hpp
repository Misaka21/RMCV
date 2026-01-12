//
// 相机外参自动标定模块
//
// 核心思想:
//   在不同云台姿态下拍摄静止标定板，
//   如果外参正确，所有帧转到World后的位置应该重合。
//   优化目标: 最小化World坐标的方差。
//
// 优化变量 (6自由度):
//   - offset_x/y/z: 相机在云台坐标系下的平移 (m)
//   - delta_roll/pitch/yaw: 相机安装姿态偏差 (rad)
//

#ifndef RMCV_EXTRINSIC_CALIB_HPP
#define RMCV_EXTRINSIC_CALIB_HPP

#include <vector>
#include <optional>
#include <chrono>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <fmt/format.h>
#include <fmt/color.h>

namespace extrinsic_calib {

// ============================================================================
// 数据结构
// ============================================================================

// 单帧采样数据
struct CalibSample {
    Eigen::Quaterniond q_imu;     // 拍照时刻的IMU四元数 (World→Imu)
    Eigen::Vector3d p_camera;     // 标定板在Camera坐标系下的位置 (PnP结果)
    int frame_id;                 // 帧编号

    CalibSample() = default;
    CalibSample(const Eigen::Quaterniond& q, const Eigen::Vector3d& p, int id)
        : q_imu(q), p_camera(p), frame_id(id) {}
};

// 待优化的外参
struct ExtrinsicParams {
    // 平移 (相机在Gimbal坐标系下的偏移，单位: m)
    double offset_x = 0.0;
    double offset_y = 0.0;
    double offset_z = 0.0;

    // 安装姿态偏差 (在基础R_camera2gimbal上的额外旋转，单位: rad)
    // 应用顺序: R_final = R_delta * R_base
    double delta_roll = 0.0;
    double delta_pitch = 0.0;
    double delta_yaw = 0.0;

    Eigen::Vector3d get_offset() const {
        return Eigen::Vector3d(offset_x, offset_y, offset_z);
    }

    Eigen::Matrix3d get_delta_rotation() const {
        // ZYX欧拉角: yaw → pitch → roll
        Eigen::AngleAxisd roll_rot(delta_roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch_rot(delta_pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yaw_rot(delta_yaw, Eigen::Vector3d::UnitZ());
        return (yaw_rot * pitch_rot * roll_rot).toRotationMatrix();
    }

    void print() const {
        fmt::print("外参:\n");
        fmt::print("  平移 (m):  x={:.4f}, y={:.4f}, z={:.4f}\n",
            offset_x, offset_y, offset_z);
        fmt::print("  偏差 (deg): roll={:.2f}, pitch={:.2f}, yaw={:.2f}\n",
            delta_roll * 180.0 / M_PI,
            delta_pitch * 180.0 / M_PI,
            delta_yaw * 180.0 / M_PI);
    }
};

// 标定结果
struct CalibResult {
    bool success = false;
    ExtrinsicParams params;

    double initial_std = 0;       // 优化前的标准差 (m)
    double final_std = 0;         // 优化后的标准差 (m)
    Eigen::Vector3d world_center; // 标定板在World中的位置

    int iterations = 0;           // 优化迭代次数
    double solve_time_ms = 0;     // 求解耗时

    // 基础外参 (用于计算最终旋转矩阵)
    Eigen::Matrix3d R_cam2gimbal_base = Eigen::Matrix3d::Identity();

    // 获取修正后的完整旋转矩阵
    Eigen::Matrix3d get_final_R_camera2gimbal() const {
        return R_cam2gimbal_base * params.get_delta_rotation();
    }

    void print() const {
        fmt::print("\n========== 标定结果 ==========\n");
        if (success) {
            fmt::print(fmt::fg(fmt::color::green), "状态: 成功!\n");
        } else {
            fmt::print(fmt::fg(fmt::color::red), "状态: 失败\n");
        }
        fmt::print("初始标准差: {:.4f} m\n", initial_std);
        fmt::print("最终标准差: {:.4f} m\n", final_std);
        fmt::print("改进: {:.1f}%\n", (1.0 - final_std / initial_std) * 100);
        fmt::print("迭代次数: {}\n", iterations);
        fmt::print("求解耗时: {:.1f} ms\n", solve_time_ms);
        fmt::print("标定板位置 (World): ({:.3f}, {:.3f}, {:.3f})\n",
            world_center.x(), world_center.y(), world_center.z());
        fmt::print("\n");
        params.print();

        // 显示修正后的完整旋转矩阵
        Eigen::Matrix3d R_final = get_final_R_camera2gimbal();
        fmt::print("\n修正后的 R_camera2gimbal (可直接复制到 camera.yaml):\n");
        fmt::print("R_camera2gimbal: [ {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f} ]\n",
            R_final(0,0), R_final(0,1), R_final(0,2),
            R_final(1,0), R_final(1,1), R_final(1,2),
            R_final(2,0), R_final(2,1), R_final(2,2));

        fmt::print("==============================\n\n");
    }
};

// 搜索范围配置
struct SearchConfig {
    // 平移范围 (m)
    double offset_range = 0.05;    // ±5cm
    double offset_step = 0.005;    // 5mm步长

    // 旋转范围 (rad)
    double angle_range = 0.2;      // ±11.5°
    double angle_step = 0.02;      // ~1.1°步长

    void print() const {
        fmt::print("搜索范围:\n");
        fmt::print("  平移: ±{:.1f}cm, 步长{:.1f}mm\n",
            offset_range * 100, offset_step * 1000);
        fmt::print("  旋转: ±{:.1f}°, 步长{:.1f}°\n",
            angle_range * 180 / M_PI, angle_step * 180 / M_PI);

        // 估算搜索空间大小
        int offset_steps = static_cast<int>(offset_range * 2 / offset_step) + 1;
        int angle_steps = static_cast<int>(angle_range * 2 / angle_step) + 1;
        long long total = static_cast<long long>(offset_steps) * offset_steps * offset_steps
                        * angle_steps * angle_steps * angle_steps;
        fmt::print("  搜索空间: {} 点 ({} x {} x {} x {} x {} x {})\n",
            total, offset_steps, offset_steps, offset_steps,
            angle_steps, angle_steps, angle_steps);
    }
};

// ============================================================================
// 外参加载 (从YAML)
// ============================================================================

struct BaseExtrinsic {
    Eigen::Matrix3d R_gimbal2imu = Eigen::Matrix3d::Identity();   // Gimbal → Imu
    Eigen::Matrix3d R_cam2gimbal = Eigen::Matrix3d::Identity();   // Camera → Gimbal (基础坐标系变换)
};

inline BaseExtrinsic load_base_extrinsic(const std::string& yaml_file = "camera.yaml") {
    BaseExtrinsic result;
    std::string full_path = std::string(CONFIG_DIR) + "/" + yaml_file;
    cv::FileStorage fs(full_path, cv::FileStorage::READ);

    if (!fs.isOpened()) {
        fmt::print(fmt::fg(fmt::color::yellow),
            "警告: 无法打开 {}, 使用默认外参\n", full_path);
        return result;
    }

    // R_gimbal2imubody
    std::vector<double> r_gimbal_data;
    fs["R_gimbal2imubody"] >> r_gimbal_data;
    if (r_gimbal_data.size() == 9) {
        result.R_gimbal2imu << r_gimbal_data[0], r_gimbal_data[1], r_gimbal_data[2],
                              r_gimbal_data[3], r_gimbal_data[4], r_gimbal_data[5],
                              r_gimbal_data[6], r_gimbal_data[7], r_gimbal_data[8];
    }

    // R_camera2gimbal
    std::vector<double> r_cam_data;
    fs["R_camera2gimbal"] >> r_cam_data;
    if (r_cam_data.size() == 9) {
        result.R_cam2gimbal << r_cam_data[0], r_cam_data[1], r_cam_data[2],
                              r_cam_data[3], r_cam_data[4], r_cam_data[5],
                              r_cam_data[6], r_cam_data[7], r_cam_data[8];
    }

    return result;
}

// ============================================================================
// 坐标变换函数
// ============================================================================

// Camera → World 变换
inline Eigen::Vector3d camera_to_world(
    const Eigen::Vector3d& p_camera,
    const Eigen::Quaterniond& q_imu,
    const BaseExtrinsic& base,
    const ExtrinsicParams& params)
{
    // 1. 构建完整的 Camera → Gimbal 旋转
    //    R_final = R_base * R_delta (先应用delta修正，再应用基础变换)
    Eigen::Matrix3d R_cam2gimbal = base.R_cam2gimbal * params.get_delta_rotation();

    // 2. Camera → Gimbal
    Eigen::Vector3d offset = params.get_offset();
    Eigen::Vector3d p_gimbal = R_cam2gimbal * p_camera + offset;

    // 3. Gimbal → Imu
    Eigen::Vector3d p_imu = base.R_gimbal2imu * p_gimbal;

    // 4. Imu → World (使用q_imu)
    Eigen::Matrix3d R_imu2world = q_imu.toRotationMatrix();
    Eigen::Vector3d p_world = R_imu2world * p_imu;

    return p_world;
}

// 计算残差 (所有帧转到World后的位置方差)
inline double compute_variance(
    const std::vector<CalibSample>& samples,
    const BaseExtrinsic& base,
    const ExtrinsicParams& params,
    Eigen::Vector3d* mean_out = nullptr)
{
    if (samples.empty()) return 1e10;

    // 计算所有点的World坐标
    std::vector<Eigen::Vector3d> world_points;
    world_points.reserve(samples.size());

    for (const auto& s : samples) {
        world_points.push_back(camera_to_world(s.p_camera, s.q_imu, base, params));
    }

    // 计算均值
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto& p : world_points) {
        mean += p;
    }
    mean /= static_cast<double>(world_points.size());

    if (mean_out) *mean_out = mean;

    // 计算方差
    double variance = 0;
    for (const auto& p : world_points) {
        variance += (p - mean).squaredNorm();
    }
    variance /= static_cast<double>(world_points.size());

    return variance;
}

// 计算标准差
inline double compute_std(
    const std::vector<CalibSample>& samples,
    const BaseExtrinsic& base,
    const ExtrinsicParams& params,
    Eigen::Vector3d* mean_out = nullptr)
{
    return std::sqrt(compute_variance(samples, base, params, mean_out));
}

// ============================================================================
// 方法1: 网格搜索
// ============================================================================

inline CalibResult grid_search(
    const std::vector<CalibSample>& samples,
    const BaseExtrinsic& base,
    const SearchConfig& config = SearchConfig(),
    bool verbose = true)
{
    if (verbose) {
        fmt::print("\n========== 网格搜索 ==========\n");
        config.print();
        fmt::print("样本数: {}\n", samples.size());
        fmt::print("开始搜索...\n\n");
    }

    auto start_time = std::chrono::steady_clock::now();

    // 初始外参 (全零)
    ExtrinsicParams initial_params;
    double initial_std = compute_std(samples, base, initial_params);

    // 最优解
    ExtrinsicParams best_params;
    double best_variance = 1e10;
    int total_iterations = 0;

    // 6维网格搜索
    for (double ox = -config.offset_range; ox <= config.offset_range; ox += config.offset_step) {
        for (double oy = -config.offset_range; oy <= config.offset_range; oy += config.offset_step) {
            for (double oz = -config.offset_range; oz <= config.offset_range; oz += config.offset_step) {
                for (double dr = -config.angle_range; dr <= config.angle_range; dr += config.angle_step) {
                    for (double dp = -config.angle_range; dp <= config.angle_range; dp += config.angle_step) {
                        for (double dy = -config.angle_range; dy <= config.angle_range; dy += config.angle_step) {
                            ExtrinsicParams params;
                            params.offset_x = ox;
                            params.offset_y = oy;
                            params.offset_z = oz;
                            params.delta_roll = dr;
                            params.delta_pitch = dp;
                            params.delta_yaw = dy;

                            double var = compute_variance(samples, base, params);
                            total_iterations++;

                            if (var < best_variance) {
                                best_variance = var;
                                best_params = params;
                            }
                        }
                    }
                }
            }
        }

        // 进度显示
        if (verbose) {
            double progress = (ox + config.offset_range) / (2 * config.offset_range);
            fmt::print("\r进度: {:.1f}%  当前最优std: {:.4f}m",
                progress * 100, std::sqrt(best_variance));
            std::cout << std::flush;
        }
    }

    auto end_time = std::chrono::steady_clock::now();
    double solve_time = std::chrono::duration<double, std::milli>(end_time - start_time).count();

    if (verbose) {
        fmt::print("\n\n");
    }

    // 构建结果
    CalibResult result;
    result.success = (best_variance < initial_std * initial_std);  // 有改进就算成功
    result.params = best_params;
    result.initial_std = initial_std;
    result.final_std = std::sqrt(best_variance);
    compute_std(samples, base, best_params, &result.world_center);
    result.iterations = total_iterations;
    result.solve_time_ms = solve_time;
    result.R_cam2gimbal_base = base.R_cam2gimbal;  // 保存基础旋转矩阵

    return result;
}

// ============================================================================
// 方法2: 两阶段搜索 (粗搜 + 细搜)
// ============================================================================

inline CalibResult two_stage_search(
    const std::vector<CalibSample>& samples,
    const BaseExtrinsic& base,
    bool verbose = true)
{
    if (verbose) {
        fmt::print("\n========== 两阶段搜索 ==========\n");
        fmt::print("样本数: {}\n\n", samples.size());
    }

    auto start_time = std::chrono::steady_clock::now();
    int total_iterations = 0;

    // 阶段1: 粗搜 (快速定位大致区域)
    SearchConfig coarse_config;
    coarse_config.offset_range = 0.05;   // ±5cm
    coarse_config.offset_step = 0.01;    // 1cm → 11³ = 1331
    coarse_config.angle_range = 0.2;     // ±11.5°
    coarse_config.angle_step = 0.04;     // ~2.3° → 11³ = 1331
    // 总计: 1331 × 1331 ≈ 1.8M 点, 约 1-2 秒

    if (verbose) {
        fmt::print("阶段1: 粗搜\n");
        coarse_config.print();
    }

    auto coarse_result = grid_search(samples, base, coarse_config, false);
    total_iterations += coarse_result.iterations;

    if (verbose) {
        fmt::print("粗搜完成, std = {:.4f}m\n\n", coarse_result.final_std);
    }

    // 阶段2: 细搜 (在粗搜最优解附近)
    // 细搜精度有限，主要靠 Ceres 精修
    SearchConfig fine_config;
    fine_config.offset_range = 0.012;    // ±1.2cm
    fine_config.offset_step = 0.002;     // 2mm → 13³ = 2,197
    fine_config.angle_range = 0.05;      // ±2.9°
    fine_config.angle_step = 0.008;      // ~0.46° → 13³ = 2,197
    // 总计: 2,197 × 2,197 ≈ 4.8M 点, 约 2-3 秒

    if (verbose) {
        fmt::print("阶段2: 细搜 (在粗搜最优解附近)\n");
        fine_config.print();
    }

    // 以粗搜结果为中心进行细搜
    ExtrinsicParams best_params = coarse_result.params;
    double best_variance = coarse_result.final_std * coarse_result.final_std;

    const auto& cp = coarse_result.params;  // 粗搜结果

    for (double ox = cp.offset_x - fine_config.offset_range;
         ox <= cp.offset_x + fine_config.offset_range;
         ox += fine_config.offset_step)
    {
        for (double oy = cp.offset_y - fine_config.offset_range;
             oy <= cp.offset_y + fine_config.offset_range;
             oy += fine_config.offset_step)
        {
            for (double oz = cp.offset_z - fine_config.offset_range;
                 oz <= cp.offset_z + fine_config.offset_range;
                 oz += fine_config.offset_step)
            {
                for (double dr = cp.delta_roll - fine_config.angle_range;
                     dr <= cp.delta_roll + fine_config.angle_range;
                     dr += fine_config.angle_step)
                {
                    for (double dp = cp.delta_pitch - fine_config.angle_range;
                         dp <= cp.delta_pitch + fine_config.angle_range;
                         dp += fine_config.angle_step)
                    {
                        for (double dy = cp.delta_yaw - fine_config.angle_range;
                             dy <= cp.delta_yaw + fine_config.angle_range;
                             dy += fine_config.angle_step)
                        {
                            ExtrinsicParams params;
                            params.offset_x = ox;
                            params.offset_y = oy;
                            params.offset_z = oz;
                            params.delta_roll = dr;
                            params.delta_pitch = dp;
                            params.delta_yaw = dy;

                            double var = compute_variance(samples, base, params);
                            total_iterations++;

                            if (var < best_variance) {
                                best_variance = var;
                                best_params = params;
                            }
                        }
                    }
                }
            }
        }
    }

    auto end_time = std::chrono::steady_clock::now();
    double solve_time = std::chrono::duration<double, std::milli>(end_time - start_time).count();

    // 构建结果
    ExtrinsicParams initial_params;
    double initial_std = compute_std(samples, base, initial_params);

    CalibResult result;
    result.success = true;
    result.params = best_params;
    result.initial_std = initial_std;
    result.final_std = std::sqrt(best_variance);
    compute_std(samples, base, best_params, &result.world_center);
    result.iterations = total_iterations;
    result.solve_time_ms = solve_time;
    result.R_cam2gimbal_base = base.R_cam2gimbal;  // 保存基础旋转矩阵

    return result;
}

// ============================================================================
// 方法3: 多轮迭代搜索 (覆盖大角度范围)
// ============================================================================

inline CalibResult multi_iteration_search(
    const std::vector<CalibSample>& samples,
    const BaseExtrinsic& base,
    int max_iterations = 5,
    bool verbose = true)
{
    if (verbose) {
        fmt::print("\n========== 多轮迭代搜索 ==========\n");
        fmt::print("样本数: {}, 最大迭代: {}\n\n", samples.size(), max_iterations);
    }

    auto start_time = std::chrono::steady_clock::now();
    int total_iterations = 0;

    // 初始外参
    ExtrinsicParams initial_params;
    double initial_std = compute_std(samples, base, initial_params);

    // 全局最优
    ExtrinsicParams global_best_params;
    double global_best_variance = 1e10;

    // 多个初始点搜索 (覆盖 ±90° 范围)
    // 对 roll, pitch, yaw 各尝试 -90°, -45°, 0°, 45°, 90°
    std::vector<double> angle_starts = {-M_PI/2, -M_PI/4, 0, M_PI/4, M_PI/2};

    if (verbose) {
        fmt::print("阶段1: 多起点粗搜 (覆盖 ±90°)\n");
        fmt::print("  每个角度测试 {} 个起点, 共 {}³ = {} 个起点\n",
            angle_starts.size(), angle_starts.size(),
            angle_starts.size() * angle_starts.size() * angle_starts.size());
    }

    // 粗搜配置 (小范围，用于快速评估每个起点)
    SearchConfig quick_config;
    quick_config.offset_range = 0.08;   // ±8cm
    quick_config.offset_step = 0.04;    // 4cm → 5 steps
    quick_config.angle_range = 0.3;     // ±17°
    quick_config.angle_step = 0.15;     // ~8.6° → 5 steps
    // 每个起点: 5^6 = 15625 点, 约 0.01 秒

    // 测试所有起点组合
    for (double start_roll : angle_starts) {
        for (double start_pitch : angle_starts) {
            for (double start_yaw : angle_starts) {
                ExtrinsicParams best_params;
                double best_variance = 1e10;

                // 在该起点附近搜索
                for (double ox = -quick_config.offset_range; ox <= quick_config.offset_range; ox += quick_config.offset_step) {
                    for (double oy = -quick_config.offset_range; oy <= quick_config.offset_range; oy += quick_config.offset_step) {
                        for (double oz = -quick_config.offset_range; oz <= quick_config.offset_range; oz += quick_config.offset_step) {
                            for (double dr = start_roll - quick_config.angle_range;
                                 dr <= start_roll + quick_config.angle_range;
                                 dr += quick_config.angle_step) {
                                for (double dp = start_pitch - quick_config.angle_range;
                                     dp <= start_pitch + quick_config.angle_range;
                                     dp += quick_config.angle_step) {
                                    for (double dy = start_yaw - quick_config.angle_range;
                                         dy <= start_yaw + quick_config.angle_range;
                                         dy += quick_config.angle_step) {
                                        ExtrinsicParams params;
                                        params.offset_x = ox;
                                        params.offset_y = oy;
                                        params.offset_z = oz;
                                        params.delta_roll = dr;
                                        params.delta_pitch = dp;
                                        params.delta_yaw = dy;

                                        double var = compute_variance(samples, base, params);
                                        total_iterations++;

                                        if (var < best_variance) {
                                            best_variance = var;
                                            best_params = params;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                // 更新全局最优
                if (best_variance < global_best_variance) {
                    global_best_variance = best_variance;
                    global_best_params = best_params;
                    if (verbose) {
                        fmt::print("  发现更优解: std={:.4f}m @ (roll={:.1f}°, pitch={:.1f}°, yaw={:.1f}°)\n",
                            std::sqrt(best_variance),
                            best_params.delta_roll * 180 / M_PI,
                            best_params.delta_pitch * 180 / M_PI,
                            best_params.delta_yaw * 180 / M_PI);
                    }
                }
            }
        }
    }

    if (verbose) {
        fmt::print("\n粗搜最优: std={:.4f}m\n", std::sqrt(global_best_variance));
        global_best_params.print();
    }

    // 阶段2: 多轮细化迭代
    if (verbose) {
        fmt::print("\n阶段2: 迭代细化 (最多 {} 轮)\n", max_iterations);
    }

    for (int iter = 0; iter < max_iterations; iter++) {
        ExtrinsicParams center = global_best_params;
        ExtrinsicParams iter_best = center;
        double iter_best_var = global_best_variance;

        // 细搜配置 (逐轮缩小范围)
        double shrink = std::pow(0.5, iter);  // 每轮缩小一半
        SearchConfig fine_config;
        fine_config.offset_range = 0.02 * shrink;   // 初始 ±2cm
        fine_config.offset_step = 0.004 * shrink;   // 初始 4mm
        fine_config.angle_range = 0.1 * shrink;     // 初始 ±5.7°
        fine_config.angle_step = 0.02 * shrink;     // 初始 1.1°

        // 在当前最优解附近细搜
        for (double ox = center.offset_x - fine_config.offset_range;
             ox <= center.offset_x + fine_config.offset_range;
             ox += fine_config.offset_step) {
            for (double oy = center.offset_y - fine_config.offset_range;
                 oy <= center.offset_y + fine_config.offset_range;
                 oy += fine_config.offset_step) {
                for (double oz = center.offset_z - fine_config.offset_range;
                     oz <= center.offset_z + fine_config.offset_range;
                     oz += fine_config.offset_step) {
                    for (double dr = center.delta_roll - fine_config.angle_range;
                         dr <= center.delta_roll + fine_config.angle_range;
                         dr += fine_config.angle_step) {
                        for (double dp = center.delta_pitch - fine_config.angle_range;
                             dp <= center.delta_pitch + fine_config.angle_range;
                             dp += fine_config.angle_step) {
                            for (double dy = center.delta_yaw - fine_config.angle_range;
                                 dy <= center.delta_yaw + fine_config.angle_range;
                                 dy += fine_config.angle_step) {
                                ExtrinsicParams params;
                                params.offset_x = ox;
                                params.offset_y = oy;
                                params.offset_z = oz;
                                params.delta_roll = dr;
                                params.delta_pitch = dp;
                                params.delta_yaw = dy;

                                double var = compute_variance(samples, base, params);
                                total_iterations++;

                                if (var < iter_best_var) {
                                    iter_best_var = var;
                                    iter_best = params;
                                }
                            }
                        }
                    }
                }
            }
        }

        // 检查是否有改进
        double improvement = (global_best_variance - iter_best_var) / global_best_variance;
        global_best_variance = iter_best_var;
        global_best_params = iter_best;

        if (verbose) {
            fmt::print("  迭代 {}: std={:.4f}m, 改进={:.2f}%\n",
                iter + 1, std::sqrt(global_best_variance), improvement * 100);
        }

        // 收敛判断
        if (improvement < 0.001) {  // < 0.1% 改进就停止
            if (verbose) {
                fmt::print("  已收敛，提前终止\n");
            }
            break;
        }
    }

    auto end_time = std::chrono::steady_clock::now();
    double solve_time = std::chrono::duration<double, std::milli>(end_time - start_time).count();

    // 构建结果
    CalibResult result;
    result.success = (global_best_variance < initial_std * initial_std);
    result.params = global_best_params;
    result.initial_std = initial_std;
    result.final_std = std::sqrt(global_best_variance);
    compute_std(samples, base, global_best_params, &result.world_center);
    result.iterations = total_iterations;
    result.solve_time_ms = solve_time;
    result.R_cam2gimbal_base = base.R_cam2gimbal;

    if (verbose) {
        fmt::print("\n多轮迭代完成:\n");
    }

    return result;
}

} // namespace extrinsic_calib

#endif // RMCV_EXTRINSIC_CALIB_HPP
