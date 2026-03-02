// Energy rune predictor implementation (2026) - 重构版

#include "buff_predictor.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <opencv2/calib3d.hpp>

#include "aimer/common/transformer/transformer.hpp"
#include "plugin/debug/logger.hpp"

namespace autobuff::predictor {

namespace {

// YOLO 关键点 3D 物理坐标 (能量机关坐标系, 单位: m)
// kpt[0-3]: 扇叶四角, kpt[4]: 扇叶中心, kpt[5]: R标区域
// 这是 slot_id=0 (θ=0, 即 +Y 方向) 的坐标
// 对于其他 slot，需要绕 Z 轴旋转 slot_id * 72°
const std::array<cv::Point3f, 6> BLADE_OBJECT_POINTS_BASE = {{
    {0.f, 0.f,    0.827f},   // kpt[0] 顶部
    {0.f, 0.127f, 0.700f},   // kpt[1] 右侧
    {0.f, 0.f,    0.573f},   // kpt[2] 底部
    {0.f, -0.127f, 0.700f},  // kpt[3] 左侧
    {0.f, 0.f,    0.700f},   // kpt[4] 扇叶中心 (= RUNE_RADIUS)
    {0.f, 0.f,    0.220f},   // kpt[5] R 标区域
}};

// 将 base keypoint 绕原点旋转 theta (在 XY 平面的 2D 旋转)
// 注意: 能量机关坐标系中扇叶在 YZ 平面分布，旋转绕 X 轴
// 但 PnP 用的 object 坐标系是平面旋转：slot 的 center 在
// (RUNE_RADIUS*cos(θ), RUNE_RADIUS*sin(θ), 0) 平面上
// 所以 keypoints 的 Y,Z 分量需要做极坐标旋转
inline cv::Point3f rotate_blade_point(const cv::Point3f& pt, double theta) {
    // pt.x = 0 (法向偏移, 保持不变)
    // pt.y, pt.z 构成扇叶平面内的坐标
    // 旋转后映射到能量机关 XY 平面:
    //   new_x = pt.z * cos(θ) - pt.y * sin(θ)   (注意 z→径向)
    //   new_y = pt.z * sin(θ) + pt.y * cos(θ)   (注意 z→径向)
    //   new_z = pt.x                             (法向)
    // 这里 pt.z 是沿径向 (距中心距离), pt.y 是切向
    float cos_t = static_cast<float>(std::cos(theta));
    float sin_t = static_cast<float>(std::sin(theta));
    return {
        pt.z * cos_t - pt.y * sin_t,  // X: 径向投影
        pt.z * sin_t + pt.y * cos_t,  // Y: 径向投影
        pt.x                           // Z: 法向 (=0)
    };
}

}  // namespace

double BuffPredictor::reduced_angle(double x) {
    return std::atan2(std::sin(x), std::cos(x));
}

void BuffPredictor::reset() {
    debouncer_.reset();
    dir_estimator_.reset();
    mode_mgr_.reset();
    const_model_.reset();
    small_model_.reset();
    large_model_.reset();
    has_last_track_ = false;
    last_track_slot_ = -1;
    last_track_phi_ = 0.0;
    last_timestamp_ = 0.0;
}

int BuffPredictor::choose_track_slot(
    const SlotDebouncer::Output& debounced,
    const BuffDetectionResult& det) const
{
    // 优先选最高置信度的稳定亮槽
    int best = -1;
    float best_conf = -1.f;
    for (int i = 0; i < NUM_SLOTS; ++i) {
        if (!debounced.slots[i].is_lit) continue;
        if (debounced.slots[i].confidence > best_conf) {
            best_conf = debounced.slots[i].confidence;
            best = i;
        }
    }
    if (best >= 0) return best;

    // 兜底: 任意有效槽位
    for (int i = 0; i < NUM_SLOTS; ++i) {
        if (det.targets[i].valid) return i;
    }
    return -1;
}

void BuffPredictor::build_ccw_rank(BuffSnapshot& snap) const {
    // 收集亮槽及其角度
    struct SlotAngle { int id; double angle; };
    std::vector<SlotAngle> lit_slots;
    lit_slots.reserve(NUM_SLOTS);

    for (int i = 0; i < NUM_SLOTS; ++i) {
        if (!snap.slots[i].is_lit) continue;
        lit_slots.push_back({i, snap.slots[i].angle});
    }

    if (lit_slots.empty()) {
        snap.ranked_count = 0;
        return;
    }

    autobuff::RotateDir dir = snap.direction;

    if (dir == autobuff::RotateDir::CW) {
        // 顺时针: 角度从大到小排序
        std::sort(lit_slots.begin(), lit_slots.end(),
                  [](const SlotAngle& a, const SlotAngle& b) {
                      return a.angle > b.angle;
                  });
    } else {
        // 逆时针或未知: 角度从小到大排序
        std::sort(lit_slots.begin(), lit_slots.end(),
                  [](const SlotAngle& a, const SlotAngle& b) {
                      return a.angle < b.angle;
                  });
    }

    snap.ranked_count = static_cast<int>(lit_slots.size());
    for (int i = 0; i < snap.ranked_count; ++i) {
        snap.ccw_lit_rank[i] = lit_slots[i].id;
    }

    snap.recommended_slot = snap.ccw_lit_rank[0];
}

BuffSnapshot BuffPredictor::predict(const BuffDetectionResult& det) {
    BuffSnapshot snap;
    snap.frame_id = det.frame_id;
    snap.timestamp = det.timestamp;
    snap.self_state = det.robot_state;

    // ============================================================
    // 1. 去抖动: 槽位稳定性过滤
    // ============================================================
    auto debounced = debouncer_.update(det, det.timestamp);

    // ============================================================
    // 2. 模式判断
    // ============================================================
    snap.mode = mode_mgr_.update(det.robot_state.aim_mode, debounced.lit_count);
    snap.lit_mask = debounced.lit_mask;
    snap.lit_count = debounced.lit_count;

    if (snap.mode == autobuff::BuffMode::UNKNOWN) {
        // 非能量机关模式，全部重置
        reset();
        snap.valid = false;
        return snap;
    }

    // ============================================================
    // 3. 选择跟踪槽位
    // ============================================================
    int track_slot = choose_track_slot(debounced, det);
    const bool has_phi = det.has_r_center() && (track_slot >= 0) &&
                         det.targets[track_slot].valid;
    const double phi_meas = has_phi ? det.targets[track_slot].angle : 0.0;

    // ============================================================
    // 4. 方向估计 (集中化, 所有模型共享)
    // ============================================================
    double dt = has_last_track_ ? (det.timestamp - last_timestamp_) : 0.0;

    if (has_phi && has_last_track_ && last_track_slot_ == track_slot) {
        dir_estimator_.feed(phi_meas, last_track_phi_, dt);
    }

    snap.direction = dir_estimator_.direction();
    int dir_sign = dir_estimator_.dir_sign();

    // ============================================================
    // 5. 模型分发与喂入
    // ============================================================
    if (has_phi) {
        switch (snap.mode) {
        case autobuff::BuffMode::SMALL_ACTIVE:
            small_model_.feed(phi_meas, det.timestamp, dir_sign);
            break;
        case autobuff::BuffMode::LARGE_INACTIVE:
            const_model_.feed(phi_meas, det.timestamp, dir_sign);
            break;
        case autobuff::BuffMode::LARGE_ACTIVE:
            large_model_.feed(phi_meas, det.timestamp, dir_sign);
            const_model_.feed(phi_meas, det.timestamp, dir_sign);
            break;
        default:
            break;
        }
    }

    // ============================================================
    // 6. 获取运动估计
    // ============================================================
    switch (snap.mode) {
    case autobuff::BuffMode::SMALL_ACTIVE:
        snap.motion = small_model_.estimate();
        break;
    case autobuff::BuffMode::LARGE_INACTIVE:
        snap.motion = const_model_.estimate();
        break;
    case autobuff::BuffMode::LARGE_ACTIVE: {
        auto large_est = large_model_.estimate();
        if (large_est.model == SpeedModel::LARGE_SINE_LSM) {
            snap.motion = large_est;
        } else {
            // 拟合尚未收敛，使用恒速兜底
            snap.motion = const_model_.estimate();
        }
        break;
    }
    default:
        snap.motion.model = SpeedModel::UNKNOWN;
        break;
    }

    // 更新跟踪状态
    if (has_phi) {
        last_track_slot_ = track_slot;
        last_track_phi_ = phi_meas;
    }
    has_last_track_ = has_phi;
    last_timestamp_ = det.timestamp;

    // ============================================================
    // 7. Group PnP: 估计 rune 平面与中心
    //    YOLO 路径: 每个扇叶贡献 6 个关键点 (1 blade + R = 7 pts ≥ 4)
    //    传统路径: 每个扇叶仅贡献 center (需 ≥4 个可见槽位)
    // ============================================================
    std::vector<cv::Point2f> img_pts;
    std::vector<cv::Point3f> obj_pts;
    img_pts.reserve(6 * NUM_SLOTS + 1);
    obj_pts.reserve(6 * NUM_SLOTS + 1);

    // R 中心作为原点 (可选)
    if (det.r_center.valid) {
        img_pts.push_back(det.r_center.center);
        obj_pts.emplace_back(0.f, 0.f, 0.f);
    }

    for (int i = 0; i < NUM_SLOTS; ++i) {
        if (!det.targets[i].valid) continue;

        // 先填充 2D 槽位信息
        snap.slots[i].valid = true;
        snap.slots[i].is_lit = debounced.slots[i].is_lit;
        snap.slots[i].confidence = det.targets[i].confidence;
        snap.slots[i].center_px = det.targets[i].center;
        snap.slots[i].angle = det.targets[i].angle;

        double theta = i * 2.0 * M_PI / NUM_SLOTS;

        if (det.targets[i].keypoint_count >= 6) {
            // YOLO 路径: 每个扇叶贡献 6 个关键点
            for (int k = 0; k < 6; ++k) {
                img_pts.push_back(det.targets[i].keypoints[k]);
                obj_pts.push_back(
                    rotate_blade_point(BLADE_OBJECT_POINTS_BASE[k], theta));
            }
        } else {
            // 传统路径: 仅用扇叶中心
            img_pts.push_back(det.targets[i].center);
            obj_pts.emplace_back(
                static_cast<float>(autobuff::RUNE_RADIUS * std::cos(theta)),
                static_cast<float>(autobuff::RUNE_RADIUS * std::sin(theta)),
                0.f
            );
        }
    }

    if (img_pts.size() < 4) {
        snap.valid = false;
        return snap;
    }

    const cv::Mat& K = aimer::tf::get_camera_matrix();
    const cv::Mat& D = aimer::tf::get_distort_coeffs();

    cv::Mat rvec, tvec;
    bool ok = cv::solvePnP(obj_pts, img_pts, K, D, rvec, tvec, false,
                           cv::SOLVEPNP_ITERATIVE);
    if (!ok) {
        snap.valid = false;
        return snap;
    }

    cv::Mat Rcv;
    cv::Rodrigues(rvec, Rcv);
    Eigen::Matrix3d R;
    R << Rcv.at<double>(0, 0), Rcv.at<double>(0, 1), Rcv.at<double>(0, 2),
         Rcv.at<double>(1, 0), Rcv.at<double>(1, 1), Rcv.at<double>(1, 2),
         Rcv.at<double>(2, 0), Rcv.at<double>(2, 1), Rcv.at<double>(2, 2);

    Eigen::Vector3d t(
        tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

    snap.center_cam = t;
    snap.center_world = aimer::tf::cam_to_world(t, det.robot_state.q_imu);

    // 轮盘法向量 (相机坐标系 +Z 方向朝向相机)
    Eigen::Vector3d n_cam = R.col(2);
    if (n_cam.z() < 0) n_cam = -n_cam;
    snap.normal_cam = n_cam.normalized();

    // 填充 3D 槽位信息
    for (int i = 0; i < NUM_SLOTS; ++i) {
        if (!snap.slots[i].valid) continue;
        double theta = i * 2.0 * M_PI / NUM_SLOTS;
        Eigen::Vector3d p_obj(
            autobuff::RUNE_RADIUS * std::cos(theta),
            autobuff::RUNE_RADIUS * std::sin(theta),
            0.0
        );
        Eigen::Vector3d p_cam = R * p_obj + t;
        snap.slots[i].pos_cam = p_cam;
        snap.slots[i].pos_world = aimer::tf::cam_to_world(p_cam, det.robot_state.q_imu);
        snap.slots[i].vec_cam = p_cam - t;
    }

    snap.valid = true;

    // ============================================================
    // 8. 逆时针排名 (双车协同)
    // ============================================================
    build_ccw_rank(snap);

    return snap;
}

}  // namespace autobuff::predictor
