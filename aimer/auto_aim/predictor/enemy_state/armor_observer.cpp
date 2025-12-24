/**
 * @file armor_observer.cpp
 * @brief 装甲板观测器实现
 */

#include "armor_observer.hpp"

#include <cmath>
#include <opencv2/imgproc.hpp>

#include "aimer/common/math/math.hpp"
#include "aimer/common/transformer/transformer.hpp"

namespace autoaim::predictor {

// 重投影误差权重 (z_to_v 越小越相信角度，越大越相信像素距离)
constexpr double DETECTOR_ERROR_PIXEL_BY_SLOPE = 2.0;

// 比赛规则: 各车型装甲板俯仰角 (弧度)
// 索引对应 ArmorNumber 枚举值 (0-8)
constexpr std::array<double, 9> ARMOR_PITCH_BY_RULE = {
    0.0, // 0: UNKNOWN
    15.0 * M_PI / 180.0, // 1: HERO (装甲板朝上15度)
    15.0 * M_PI / 180.0, // 2: ENGINEER
    15.0 * M_PI / 180.0, // 3: INFANTRY_3
    15.0 * M_PI / 180.0, // 4: INFANTRY_4
    15.0 * M_PI / 180.0, // 5: INFANTRY_5
    -15.0 * M_PI / 180.0, // 6: OUTPOST(装甲板朝下15度)
    15.0 * M_PI / 180.0, // 7: SENTRY
    15.0 * M_PI / 180.0 // 8: BASE
};

// PnP 解算的俯仰角超过此阈值时，不使用三分法优化
constexpr double ARMOR_PITCH_MAX_FOR_FIT = 30.0 * M_PI / 180.0;

const ArmorObservationTable& ArmorObserver::observe(
    const DetectionResult& detection,
    double timestamp
) {
    table_.clear();
    table_.set_frame(timestamp, ++frame_id_);

    const auto& q_imu = detection.state.q_imu;

    for (const auto& armor : detection.armors) {
        auto obs = solve_pnp(armor, timestamp, q_imu);
        if (obs.valid) {
            table_.add(obs);
        }
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
    // 物体坐标系: x前(指向相机), y左, z上
    // 法向量在物体坐标系是 [1, 0, 0] (x轴方向)
    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec, rotation_matrix);
    Eigen::Vector3d normal_cam(
        rotation_matrix.at<double>(0, 0),
        rotation_matrix.at<double>(1, 0),
        rotation_matrix.at<double>(2, 0)
    );

    // 计算初始 z_to_v (PnP 结果)
    double z_to_v_raw = compute_z_to_v(pos_cam, normal_cam);

    // 畸变矫正四角点
    std::array<cv::Point2f, 4> pus = undistort_points(armor.landmarks);

    // 获取该车型的装甲板俯仰角 (比赛规则)
    int armor_number_idx = static_cast<int>(armor.number);
    double armor_pitch = (armor_number_idx >= 0 && armor_number_idx < 9)
        ? ARMOR_PITCH_BY_RULE[armor_number_idx]
        : 0.0;

    // 计算 PnP 解算的俯仰角 (用于判断是否需要跳过三分法)
    double pnp_pitch = std::atan2(-normal_cam.y(), normal_cam.z());  // 从法向量提取俯仰

    // 三分法优化 z_to_v
    // 如果 PnP 俯仰角过大 (说明 PnP 可能不准)，跳过三分法
    double z_to_v = z_to_v_raw;
    if (std::abs(pnp_pitch) < ARMOR_PITCH_MAX_FOR_FIT) {
        z_to_v = fit_z_to_v(pos_cam, armor.type, armor_pitch, pus, z_to_v_raw);
    }

    // ========== 坐标变换: 相机系 → 世界系 ==========
    Eigen::Vector3d pos_world = tf::cam_to_world(pos_cam, q_imu);
    Eigen::Vector3d normal_world = tf::vector<tf::Frame::Camera, tf::Frame::World>(normal_cam, q_imu);

    // 计算观测向量 (世界系)
    Eigen::Vector4d z = compute_observation(pos_world, normal_world);

    // 构建观测结果 (世界系)
    obs = ArmorObservation::from_detection(armor, pos_world, z, z_to_v, timestamp, pus);

    return obs;
}

Eigen::Vector4d ArmorObserver::compute_observation(
    const Eigen::Vector3d& pos_world,
    const Eigen::Vector3d& normal_world
) {
    Eigen::Vector4d z;

    // 计算球坐标 (世界系)
    double dist = pos_world.norm();
    double yaw = std::atan2(pos_world.y(), pos_world.x());
    double pitch = std::atan2(pos_world.z(), std::sqrt(pos_world.x() * pos_world.x() + pos_world.y() * pos_world.y()));

    // 装甲板朝向角 (法向量在 xy 平面的投影角度)
    double armor_yaw = std::atan2(normal_world.y(), normal_world.x());

    z << yaw, pitch, dist, armor_yaw;

    return z;
}

double ArmorObserver::compute_z_to_v(
    const Eigen::Vector3d& pos_cam,
    const Eigen::Vector3d& normal_cam
) {
    // 视线方向 (从相机指向装甲板)
    Eigen::Vector3d view_dir = pos_cam.normalized();

    // 装甲板法向量应该大致指向相机
    double cos_angle = -normal_cam.dot(view_dir);

    // 返回夹角 (弧度)
    return std::acos(std::clamp(cos_angle, -1.0, 1.0));
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

std::array<cv::Point2f, 4> ArmorObserver::project_armor_corners(
    const Eigen::Vector3d& pos,
    ArmorType type,
    double pitch,
    double z_to_v
) {
    // 装甲板尺寸
    double w = (type == ArmorType::LARGE) ? LARGE_ARMOR_WIDTH : SMALL_ARMOR_WIDTH;
    double h = (type == ArmorType::LARGE) ? LARGE_ARMOR_HEIGHT : SMALL_ARMOR_HEIGHT;

    // 相机 z 轴在 xy 平面的投影方向 (视线方向)
    Eigen::Vector2d view_dir_2d(pos.x(), pos.y());
    view_dir_2d.normalize();

    // 装甲板法向量方向 (在 xy 平面)
    // z_to_v 是法向量与视线的夹角
    // 法向量 = 视线旋转 (π - z_to_v) 得到 (因为法向量指向相机)
    Eigen::Vector2d normal_2d = math::rotate(view_dir_2d, M_PI - z_to_v);

    // 装甲板 x 轴方向 (水平，垂直于法向量)
    Eigen::Vector3d x_axis;
    Eigen::Vector2d x_2d = math::rotate(normal_2d, M_PI / 2);
    x_axis << x_2d.x(), x_2d.y(), 0.0;

    // 装甲板 y 轴方向 (竖直，考虑俯仰)
    Eigen::Vector3d y_axis;
    y_axis << -normal_2d.x() * std::sin(pitch),
              -normal_2d.y() * std::sin(pitch),
              std::cos(pitch);

    // 计算四角点 (相机坐标系)
    // 顺序: 左下、左上、右上、右下
    std::vector<Eigen::Vector3d> corners(4);
    corners[0] = pos + x_axis * (w / 2) + y_axis * (-h / 2);  // 左下
    corners[1] = pos + x_axis * (w / 2) + y_axis * (h / 2);   // 左上
    corners[2] = pos + x_axis * (-w / 2) + y_axis * (h / 2);  // 右上
    corners[3] = pos + x_axis * (-w / 2) + y_axis * (-h / 2); // 右下

    // 投影到图像
    const cv::Mat& camera_matrix = tf::get_camera_matrix();
    double fx = camera_matrix.at<double>(0, 0);
    double fy = camera_matrix.at<double>(1, 1);
    double cx = camera_matrix.at<double>(0, 2);
    double cy = camera_matrix.at<double>(1, 2);

    std::array<cv::Point2f, 4> result;
    for (int i = 0; i < 4; ++i) {
        // 相机坐标系: x右, y下, z前
        double u = fx * corners[i].x() / corners[i].z() + cx;
        double v = fy * corners[i].y() / corners[i].z() + cy;
        result[i] = cv::Point2f(static_cast<float>(u), static_cast<float>(v));
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

double ArmorObserver::fit_z_to_v(
    const Eigen::Vector3d& pos,
    ArmorType type,
    double pitch,
    const std::array<cv::Point2f, 4>& pus,
    double z_to_v_init
) {
    // 方案1: 缩小搜索范围 (以初始值或上一帧结果为中心)
    double center = z_to_v_init + M_PI;  // 转换到 [0, 2π] 范围
    if (has_last_z_to_v_) {
        // 方案3: 用上一帧结果作为中心，搜索范围更小
        center = last_z_to_v_;
    }

    // 搜索范围: ±45度
    constexpr double SEARCH_MARGIN = M_PI / 4;
    double z_min = std::max(M_PI / 2, center - SEARCH_MARGIN);
    double z_max = std::min(M_PI * 3 / 2, center + SEARCH_MARGIN);

    // 代价函数
    auto cost_func = [&](double z_to_v) {
        auto projected = project_armor_corners(pos, type, pitch, z_to_v);
        return compute_reprojection_cost(projected, pus, z_to_v_init);
    };

    // 方案5: 带提前终止的三分搜索
    constexpr double PHI = 0.6180339887498949;
    constexpr double MIN_INTERVAL = 0.01;  // 约 0.5度，提前终止阈值

    double left = z_min, right = z_max;
    double ml_cost = 0, mr_cost = 0;
    int reserved = -1;

    for (int i = 0; i < FIT_Z_TO_V_ITERATIONS; ++i) {
        // 提前终止: 区间足够小
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

    double best_z_to_v = (left + right) / 2.0;

    // 更新上一帧记录 (方案3)
    last_z_to_v_ = best_z_to_v;
    has_last_z_to_v_ = true;

    // 规范化到 (-π, π]
    return math::reduced_angle(best_z_to_v - M_PI);
}

}  // namespace autoaim::predictor
