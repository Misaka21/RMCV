/**
 * @file armor_observer.cpp
 * @brief 装甲板观测器实现
 */

#include "armor_observer.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <numeric>
#include <opencv2/imgproc.hpp>

#include "aimer/common/math/math.hpp"
#include "aimer/common/transformer/transformer.hpp"
#include "plugin/param/runtime_parameter.hpp"

namespace autoaim::predictor {

// 重投影误差权重 (z_to_v 越小越相信角度，越大越相信像素距离)
constexpr double DETECTOR_ERROR_PIXEL_BY_SLOPE = 2.0;

// 比赛规则: 各车型装甲板俯仰角 (弧度)
// 索引对应 ArmorNumber 枚举值 (0-8)
// 注意: 负值表示装甲板上沿向后倾斜 (与 rm.cv.fans 一致)
constexpr std::array<double, 9> ARMOR_PITCH_BY_RULE = {
    0.0, // 0: UNKNOWN
    -15.0 * M_PI / 180.0, // 1: HERO
    -15.0 * M_PI / 180.0, // 2: ENGINEER
    -15.0 * M_PI / 180.0, // 3: INFANTRY_3
    -15.0 * M_PI / 180.0, // 4: INFANTRY_4
    -15.0 * M_PI / 180.0, // 5: INFANTRY_5
    15.0 * M_PI / 180.0, // 6: OUTPOST (前哨站相反)
    -15.0 * M_PI / 180.0, // 7: SENTRY
    -15.0 * M_PI / 180.0 // 8: BASE
};

// PnP 解算的俯仰角超过此阈值时，不使用三分法优化
constexpr double ARMOR_PITCH_MAX_FOR_FIT = 30.0 * M_PI / 180.0;

// 位置相近性合并阈值 (米)
// 同一辆车的两块装甲板距离通常 < 1m，两辆车之间通常 > 2m
constexpr double SAME_VEHICLE_DISTANCE_THRESHOLD = 1.5;

const ArmorObservationTable& ArmorObserver::observe(
    const DetectionResult& detection,
    double timestamp
) {
    table_.clear();
    table_.set_frame(timestamp, ++frame_id_);

    const auto& q_imu = detection.state.q_imu;

    // 第一步：收集所有有效的观测
    std::vector<ArmorObservation> observations;
    for (const auto& armor : detection.armors) {
        auto obs = solve_pnp(armor, timestamp, q_imu);
        if (obs.valid) {
            observations.push_back(obs);
        }
    }

    // 第二步：双装甲板联合三分法优化 + 智能合并
    // 对于可能是同一车的装甲板对，先做联合优化，再检查几何约束决定是否合并
    bool use_double_fit = runtime_param::get_param<bool>("AutoAim.Predictor.use_double_z_fit");
    if (observations.size() >= 2) {
        // 收集所有可能的装甲板对 (距离在合理范围内)
        std::vector<std::pair<size_t, size_t>> candidate_pairs;
        for (size_t i = 0; i < observations.size(); ++i) {
            for (size_t j = i + 1; j < observations.size(); ++j) {
                double dist = (observations[i].pos - observations[j].pos).norm();
                // 同一车的两块装甲板距离通常在 0.25m ~ 1.2m
                if (dist >= 0.25 && dist <= 1.2) {
                    candidate_pairs.emplace_back(i, j);
                }
            }
        }

        // 对每个候选对进行处理
        for (auto [idx_i, idx_j] : candidate_pairs) {
            auto& obs_i = observations[idx_i];
            auto& obs_j = observations[idx_j];

            // 判断左右 (根据 z_to_v，更负的是左边)
            bool swap = obs_i.z_to_v > obs_j.z_to_v;
            auto& obs_l = swap ? obs_j : obs_i;
            auto& obs_r = swap ? obs_i : obs_j;

            // 保存原始值用于比较
            double z_to_l_before = obs_l.z_to_v;
            double z_to_r_before = obs_r.z_to_v;

            // 联合三分法优化 (利用 90° 约束)
            if (use_double_fit) {
                double z_to_l = fit_double_z_to_l(obs_l, obs_r, obs_l.z_to_v, q_imu);
                double z_to_r = z_to_l + M_PI / 2;

                // 更新 z_to_v
                obs_l.z_to_v = z_to_l;
                obs_r.z_to_v = z_to_r;

                // 更新 armor_yaw (z[3])
                double camera_yaw = std::atan2(
                    get_camera_z_i2(q_imu).y(),
                    get_camera_z_i2(q_imu).x()
                );
                obs_l.z[3] = z_to_l + camera_yaw;
                obs_r.z[3] = z_to_r + camera_yaw;
            }

            // 用优化后的值检查几何约束
            double yaw_i = obs_i.z[3];
            double yaw_j = obs_j.z[3];
            double yaw_diff = std::abs(math::angle_diff(yaw_i, yaw_j));

            // 检查1: 朝向角差是否接近 90°
            bool angle_ok = (yaw_diff > 70.0 * M_PI / 180.0 && yaw_diff < 110.0 * M_PI / 180.0);

            // 检查2: 估算旋转中心是否一致
            double r_est = 0.26;
            Eigen::Vector3d center_i = obs_i.pos + r_est * Eigen::Vector3d(
                std::cos(yaw_i), std::sin(yaw_i), 0);
            Eigen::Vector3d center_j = obs_j.pos + r_est * Eigen::Vector3d(
                std::cos(yaw_j), std::sin(yaw_j), 0);
            double center_dist = (center_i - center_j).head<2>().norm();

            bool center_ok = (center_dist < 0.25);

            double dist = (obs_i.pos - obs_j.pos).norm();

            // 如果通过几何检查且 target_id 不同，合并
            if (angle_ok && center_ok && obs_i.target_id != obs_j.target_id) {
                // 使用距离近的那个的 target_id
                int merged_id = (obs_i.distance() < obs_j.distance())
                    ? obs_i.target_id : obs_j.target_id;

                fmt::print(fmt::fg(fmt::color::yellow),
                    "[ArmorObserver] 合并同车装甲板: T{} + T{} → T{} "
                    "(dist={:.2f}m, yaw_diff={:.1f}°, center_dist={:.2f}m)\n",
                    obs_i.target_id, obs_j.target_id, merged_id,
                    dist, yaw_diff * 180.0 / M_PI, center_dist);

                obs_i.target_id = merged_id;
                obs_j.target_id = merged_id;

                // DEBUG: 打印优化结果
                fmt::print(fmt::fg(fmt::color::green),
                    "[DoubleZ] T{}: z_to_l {:.1f}° → {:.1f}°, z_to_r {:.1f}° → {:.1f}°\n",
                    merged_id,
                    z_to_l_before * 180.0 / M_PI, obs_l.z_to_v * 180.0 / M_PI,
                    z_to_r_before * 180.0 / M_PI, obs_r.z_to_v * 180.0 / M_PI);
            } else if (obs_i.target_id == obs_j.target_id && use_double_fit) {
                // 同一 target_id，保留优化结果
                fmt::print(fmt::fg(fmt::color::green),
                    "[DoubleZ] T{}: z_to_l {:.1f}° → {:.1f}°, z_to_r {:.1f}° → {:.1f}°\n",
                    obs_i.target_id,
                    z_to_l_before * 180.0 / M_PI, obs_l.z_to_v * 180.0 / M_PI,
                    z_to_r_before * 180.0 / M_PI, obs_r.z_to_v * 180.0 / M_PI);
            } else if (use_double_fit) {
                // 几何不匹配，恢复原始值
                obs_l.z_to_v = z_to_l_before;
                obs_r.z_to_v = z_to_r_before;
                // 恢复 armor_yaw
                double camera_yaw = std::atan2(
                    get_camera_z_i2(q_imu).y(),
                    get_camera_z_i2(q_imu).x()
                );
                obs_l.z[3] = z_to_l_before + camera_yaw;
                obs_r.z[3] = z_to_r_before + camera_yaw;
            }
        }
    }

    // 第三步：添加到表
    for (const auto& obs : observations) {
        table_.add(obs);
    }

    return table_;
}

ArmorObservation ArmorObserver::solve_pnp(
    const autoaim::DetectedArmor& armor,
    double timestamp,
    const Eigen::Quaterniond& q_imu
) {
    ArmorObservation obs;
    obs.valid = false;

    // 检查四角点数量
    if (armor.landmarks.size() != 4) {
        return obs;
    }

    // 使用 DetectedArmor 的模板点 (统一定义)
    std::vector<cv::Point3f> object_points = armor.object_points();

    // 2D 图像点
    std::vector<cv::Point2f> image_points(armor.landmarks.begin(), armor.landmarks.end());

    // 从 tf 模块获取相机内参
    const cv::Mat& camera_matrix = tf::get_camera_matrix();
    const cv::Mat& dist_coeffs = tf::get_distort_coeffs();

    // PnP 解算
    cv::Mat rvec, tvec;
    bool success = cv::solvePnP(
        object_points,
        image_points,
        camera_matrix,
        dist_coeffs,
        rvec,
        tvec,
        false,
        cv::SOLVEPNP_IPPE
    );

    if (!success) {
        return obs;
    }

    // 提取位置 (相机坐标系)
    Eigen::Vector3d pos_cam(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

    // 计算装甲板法向量 (相机坐标系)
    // 物体坐标系: x水平(右), y垂直(上), z=0平面
    // 点序为逆时针(从+Z看)，正对时相机在物体+Z侧
    // 前表面法向量是+Z，即R.col(2)
    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec, rotation_matrix);
    Eigen::Vector3d normal_cam(
        rotation_matrix.at<double>(0, 2),
        rotation_matrix.at<double>(1, 2),
        rotation_matrix.at<double>(2, 2)
    );

    // ========== 坐标变换: 相机系 → 世界系 ==========
    Eigen::Vector3d pos_world = tf::cam_to_world(pos_cam, q_imu);
    // 装甲板"背面"法向量 (指向远离相机的方向, 即从装甲板指向车体中心)
    // 注: 这里取负号是为了让 z_to_v = 0 表示正对
    // armor_yaw 是从装甲板指向中心的方向 (INWARD)
    Eigen::Vector3d normal_world = tf::vector<tf::Frame::Camera, tf::Frame::World>(-normal_cam, q_imu);

    // 畸变矫正四角点
    std::array<cv::Point2f, 4> pus = undistort_points(armor.landmarks);

    // 获取该车型的装甲板俯仰角 (比赛规则)
    int armor_number_idx = static_cast<int>(armor.number);
    double armor_pitch = (armor_number_idx >= 0 && armor_number_idx < 9)
        ? ARMOR_PITCH_BY_RULE[armor_number_idx]
        : 0.0;

    // 计算 PnP 解算的俯仰角 (用于判断是否需要跳过三分法)
    double horizontal = std::sqrt(normal_cam.x() * normal_cam.x() + normal_cam.z() * normal_cam.z());
    double pnp_pitch = std::atan2(-normal_cam.y(), horizontal);

    // ========== 计算初始 z_to_v (世界坐标系方法) ==========
    // z_to_v = 装甲板法向量相对于相机前向的旋转角 (绕世界 Z 轴)
    Eigen::Vector2d camera_z_i2 = get_camera_z_i2(q_imu);
    Eigen::Vector2d normal_xy(normal_world.x(), normal_world.y());
    double normal_xy_norm = normal_xy.norm();
    double z_to_v_raw = 0.0;
    if (normal_xy_norm > 1e-6) {
        normal_xy /= normal_xy_norm;
        // 计算从 camera_z_i2 到 normal_xy 的有向角
        z_to_v_raw = std::atan2(
            camera_z_i2.x() * normal_xy.y() - camera_z_i2.y() * normal_xy.x(),
            camera_z_i2.dot(normal_xy)
        );
    }

    // 三分法优化 z_to_v (仅当俯仰角不太大时)
    double z_to_v = z_to_v_raw;
    if (std::abs(pnp_pitch) < ARMOR_PITCH_MAX_FOR_FIT) {
        z_to_v = fit_z_to_v(pos_world, armor.type, armor_pitch, pus, z_to_v_raw, q_imu);
    }

    // DEBUG: 详细的法向量调试信息
    // normal_cam: PnP 解算的法向量 (相机坐标系)
    //   - 正对时应该是 (0, 0, 1) 指向相机
    //   - 侧面 60° 时应该是 (±0.866, 0, 0.5)
    // 3D 夹角: 法向量与相机 Z 轴的夹角 (包含俯仰分量)
    double cos_3d_angle = -normal_cam.z();  // normal_cam 指向相机时是负的
    double angle_3d = std::acos(std::clamp(cos_3d_angle, -1.0, 1.0)) * 180.0 / M_PI;

    fmt::print(fmt::fg(fmt::color::cyan),
        "[PnP] normal_cam: ({:.3f}, {:.3f}, {:.3f}) → 3D夹角: {:.1f}°\n",
        normal_cam.x(), normal_cam.y(), normal_cam.z(), angle_3d);
    fmt::print(fmt::fg(fmt::color::orange),
        "[z_to_v] raw: {:.1f}° → fit: {:.1f}° (pnp_pitch: {:.1f}°)\n",
        z_to_v_raw * 180.0 / M_PI, z_to_v * 180.0 / M_PI, pnp_pitch * 180.0 / M_PI);

    // 使用稳定的 z_to_v 计算 armor_yaw
    // 参考 rm.cv.fans: armor_yaw = z_to_v + camera_z_i_yaw
    double camera_yaw = std::atan2(camera_z_i2.y(), camera_z_i2.x());
    double armor_yaw = z_to_v + camera_yaw;

    // 计算观测向量 (世界系)
    double dist = pos_world.norm();
    double pos_yaw = std::atan2(pos_world.y(), pos_world.x());
    double pos_pitch = std::atan2(pos_world.z(), pos_world.head<2>().norm());
    Eigen::Vector4d z;
    z << pos_yaw, pos_pitch, dist, armor_yaw;

    // 构建观测结果 (世界系)
    // 注意: z_to_v 是世界坐标系下的角度 (相对于相机前向)
    obs = ArmorObservation::from_detection(armor, pos_world, z, z_to_v, z_to_v_raw, timestamp, pus);

    return obs;
}

// ==================== 三分法优化 z_to_v ====================

std::array<cv::Point2f, 4> ArmorObserver::undistort_points(
    const std::vector<cv::Point2f>& pts
) {
    std::array<cv::Point2f, 4> result = {};
    if (pts.size() != 4) return result;

    const cv::Mat& camera_matrix = tf::get_camera_matrix();
    const cv::Mat& dist_coeffs = tf::get_distort_coeffs();

    std::vector<cv::Point2f> undistorted;
    cv::undistortPoints(pts, undistorted, camera_matrix, dist_coeffs, cv::noArray(), camera_matrix);

    for (int i = 0; i < 4; ++i) {
        result[i] = undistorted[i];
    }

    return result;
}

/**
 * @brief 获取相机 Z 轴在世界 XY 平面的投影 (归一化)
 *
 * 这是相机"前向"在地面平面上的方向，用作 z_to_v 旋转的参考方向
 * 参考 rm.cv.fans: converter->get_camera_z_i2()
 */
Eigen::Vector2d ArmorObserver::get_camera_z_i2(const Eigen::Quaterniond& q_imu) {
    // 相机 Z 轴在相机坐标系是 (0, 0, 1)
    // 变换到世界坐标系
    Eigen::Vector3d camera_z_world = tf::vector<tf::Frame::Camera, tf::Frame::World>(
        Eigen::Vector3d(0, 0, 1), q_imu
    );

    // 投影到世界 XY 平面并归一化
    Eigen::Vector2d z_i2(camera_z_world.x(), camera_z_world.y());
    double norm = z_i2.norm();
    if (norm < 1e-6) {
        // 相机正对上/下，退化情况，使用世界 X 轴
        return Eigen::Vector2d(1.0, 0.0);
    }
    return z_i2 / norm;
}

/**
 * @brief 给定 z_to_v 计算装甲板四角点在图像上的投影 (世界坐标系方法)
 *
 * 参考 rm.cv.fans: radial_armor_corners + radial_armor_pts
 *
 * @param pos_world 装甲板中心 (世界坐标系)
 * @param type 装甲板类型
 * @param pitch 装甲板俯仰角 (规则定义)
 * @param z_to_v 装甲板法向量相对于相机前向的旋转角 (绕世界 Z 轴)
 * @param q_imu IMU 四元数
 * @return 投影的四角点 (像素坐标)
 */
std::array<cv::Point2f, 4> ArmorObserver::project_armor_corners(
    const Eigen::Vector3d& pos_world,
    ArmorType type,
    double pitch,
    double z_to_v,
    const Eigen::Quaterniond& q_imu
) {
    // 装甲板尺寸
    double w = (type == ArmorType::LARGE) ? LARGE_ARMOR_WIDTH : SMALL_ARMOR_WIDTH;
    double h = (type == ArmorType::LARGE) ? LARGE_ARMOR_HEIGHT : SMALL_ARMOR_HEIGHT;

    // 获取相机前向在世界 XY 平面的投影
    Eigen::Vector2d camera_z_i2 = get_camera_z_i2(q_imu);

    // 装甲板法向量在世界 XY 平面的方向 = 相机前向旋转 z_to_v
    Eigen::Vector2d radius_norm = math::rotate(camera_z_i2, z_to_v);

    // 装甲板 X 轴 (水平方向，垂直于法向量)
    Eigen::Vector3d x_axis;
    Eigen::Vector2d x_2d = math::rotate(radius_norm, M_PI / 2);
    x_axis << x_2d.x(), x_2d.y(), 0.0;

    // 装甲板 Y 轴 (竖直方向，考虑俯仰角)
    // 与 rm.cv.fans 的 radial_armor_corners 一致
    Eigen::Vector3d y_axis;
    y_axis << -radius_norm.x() * std::sin(pitch),
              -radius_norm.y() * std::sin(pitch),
              std::cos(pitch);

    // 计算四角点 (世界坐标系)
    // 顺序: 左上、左下、右下、右上 (逆时针，从相机看)
    std::array<Eigen::Vector3d, 4> corners_world;
    corners_world[0] = pos_world + x_axis * (w / 2) + y_axis * (h / 2);    // 左上
    corners_world[1] = pos_world + x_axis * (w / 2) - y_axis * (h / 2);    // 左下
    corners_world[2] = pos_world - x_axis * (w / 2) - y_axis * (h / 2);    // 右下
    corners_world[3] = pos_world - x_axis * (w / 2) + y_axis * (h / 2);    // 右上

    // 变换到相机坐标系并投影
    const cv::Mat& camera_matrix = tf::get_camera_matrix();
    double fx = camera_matrix.at<double>(0, 0);
    double fy = camera_matrix.at<double>(1, 1);
    double cx = camera_matrix.at<double>(0, 2);
    double cy = camera_matrix.at<double>(1, 2);

    std::array<cv::Point2f, 4> result;
    for (int i = 0; i < 4; ++i) {
        // 世界坐标 → 相机坐标
        Eigen::Vector3d p_cam = tf::world_to_camera(corners_world[i], q_imu);

        // 投影到像素 (相机坐标系: x右, y下, z前)
        if (p_cam.z() > 0.01) {
            double u = fx * p_cam.x() / p_cam.z() + cx;
            double v = fy * p_cam.y() / p_cam.z() + cy;
            result[i] = cv::Point2f(static_cast<float>(u), static_cast<float>(v));
        } else {
            // 点在相机后方，返回无效值
            result[i] = cv::Point2f(-1, -1);
        }
    }

    return result;
}

double ArmorObserver::compute_reprojection_cost(
    const std::array<cv::Point2f, 4>& projected,
    const std::array<cv::Point2f, 4>& detected,
    double z_to_v
) {
    // 参考 rm.cv.fans 的 get_pts_cost
    double cost = 0.0;

    for (int i = 0; i < 4; ++i) {
        int p = (i + 1) % 4;

        // 标准边 (投影)
        Eigen::Vector2d ref_d(projected[p].x - projected[i].x, projected[p].y - projected[i].y);
        // 检测边
        Eigen::Vector2d pt_d(detected[p].x - detected[i].x, detected[p].y - detected[i].y);

        double ref_norm = ref_d.norm();
        if (ref_norm < 1e-6) continue;

        // 像素距离代价 (起点差 + 长度差)
        double pixel_dis =
            (0.5 * ((Eigen::Vector2d(projected[i].x - detected[i].x, projected[i].y - detected[i].y).norm()
                   + Eigen::Vector2d(projected[p].x - detected[p].x, projected[p].y - detected[p].y).norm()))
             + std::fabs(ref_norm - pt_d.norm()))
            / ref_norm;

        // 角度差代价
        double angular_dis = math::get_abs_angle(ref_d, pt_d);

        // 加权组合 (z_to_v 越小，越相信角度; 越大，越相信像素)
        double inclined = z_to_v;  // 作为权重
        double cost_i = math::sq(pixel_dis * std::sin(inclined))
                      + math::sq(angular_dis * std::cos(inclined)) * DETECTOR_ERROR_PIXEL_BY_SLOPE;

        cost += std::sqrt(cost_i);
    }

    return cost;
}

/**
 * @brief 三分搜索找到最优的 z_to_v (世界坐标系方法)
 *
 * z_to_v 是装甲板法向量相对于相机前向的旋转角 (绕世界 Z 轴)
 * - z_to_v = 0: 装甲板正对相机
 * - z_to_v > 0: 装甲板法向量逆时针偏离相机前向
 * - z_to_v < 0: 装甲板法向量顺时针偏离相机前向
 */
double ArmorObserver::fit_z_to_v(
    const Eigen::Vector3d& pos_world,
    ArmorType type,
    double pitch,
    const std::array<cv::Point2f, 4>& pus,
    double z_to_v_init,
    const Eigen::Quaterniond& q_imu
) {
    // 搜索范围: 初始值 ±45度
    // z_to_v 范围是 [-π, π]，正对时接近 0
    constexpr double SEARCH_MARGIN = M_PI;  // 45度
    double z_min = z_to_v_init - SEARCH_MARGIN;
    double z_max = z_to_v_init + SEARCH_MARGIN;

    // 限制在可见范围 [-π/2, π/2]（超过 90 度就看不到正面了）
    z_min = std::max(-M_PI / 2, z_min);
    z_max = std::min(M_PI / 2, z_max);

    // 确保有效搜索范围
    if (z_max <= z_min + 0.02) {
        return z_to_v_init;
    }

    // 代价函数
    auto cost_func = [&](double z_to_v) {
        auto projected = project_armor_corners(pos_world, type, pitch, z_to_v, q_imu);
        return compute_reprojection_cost(projected, pus, std::abs(z_to_v_init));
    };

    // 黄金分割搜索
    constexpr double PHI = 0.6180339887498949;
    constexpr double MIN_INTERVAL = 0.01;  // 约 0.5度

    double left = z_min, right = z_max;
    double ml_cost = 0, mr_cost = 0;
    int reserved = -1;

    for (int i = 0; i < FIT_Z_TO_V_ITERATIONS; ++i) {
        if (right - left < MIN_INTERVAL) break;

        double ml = left + (right - left) * (1.0 - PHI);
        double mr = left + (right - left) * PHI;

        if (reserved != 0) ml_cost = cost_func(ml);
        if (reserved != 1) mr_cost = cost_func(mr);

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

    return (left + right) / 2.0;
}

/**
 * @brief 双装甲板联合三分法
 *
 * 参考 rm.cv.fans: fit_double_z_to_l / DoubleCost
 * 利用"两块装甲板相差 90°"的几何约束，联合优化角度
 *
 * 核心思想:
 * - 左边装甲板 z_to_v = z_to_l
 * - 右边装甲板 z_to_v = z_to_l + π/2
 * - 找到使两块装甲板总重投影误差最小的 z_to_l
 */
double ArmorObserver::fit_double_z_to_l(
    const ArmorObservation& obs0,
    const ArmorObservation& obs1,
    double z_to_l_init,
    const Eigen::Quaterniond& q_imu
) {
    // 搜索范围 (rm.cv.fans: M_PI - must_not_see_angle 到 M_PI + must_not_see_angle - angle_between_armors)
    // 简化: ±60度范围
    constexpr double SEARCH_MARGIN = M_PI / 3;  // 60度
    double z_min = z_to_l_init - SEARCH_MARGIN;
    double z_max = z_to_l_init + SEARCH_MARGIN;

    // 限制在合理范围
    z_min = std::max(-M_PI / 2, z_min);
    z_max = std::min(M_PI / 2, z_max);

    if (z_max <= z_min + 0.02) {
        return z_to_l_init;
    }

    // 获取装甲板参数
    ArmorType type0 = obs0.type;
    ArmorType type1 = obs1.type;

    // 假设两块装甲板俯仰角相同 (同一辆车)
    double pitch = -15.0 * M_PI / 180.0;  // 默认俯仰角

    // 代价函数: 两块装甲板的总重投影误差
    auto cost_func = [&](double z_to_l) {
        // 左边装甲板
        auto projected0 = project_armor_corners(obs0.pos, type0, pitch, z_to_l, q_imu);
        double cost0 = compute_reprojection_cost(projected0, obs0.pus, std::abs(z_to_l_init));

        // 右边装甲板 (相差 90°)
        double z_to_r = z_to_l + M_PI / 2;
        auto projected1 = project_armor_corners(obs1.pos, type1, pitch, z_to_r, q_imu);
        double cost1 = compute_reprojection_cost(projected1, obs1.pus, std::abs(z_to_l_init + M_PI / 2));

        return cost0 + cost1;
    };

    // 黄金分割搜索
    constexpr double PHI = 0.6180339887498949;
    constexpr double MIN_INTERVAL = 0.01;

    double left = z_min, right = z_max;
    double ml_cost = 0, mr_cost = 0;
    int reserved = -1;

    for (int i = 0; i < FIT_Z_TO_V_ITERATIONS; ++i) {
        if (right - left < MIN_INTERVAL) break;

        double ml = left + (right - left) * (1.0 - PHI);
        double mr = left + (right - left) * PHI;

        if (reserved != 0) ml_cost = cost_func(ml);
        if (reserved != 1) mr_cost = cost_func(mr);

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

    return (left + right) / 2.0;
}

}  // namespace autoaim::predictor
