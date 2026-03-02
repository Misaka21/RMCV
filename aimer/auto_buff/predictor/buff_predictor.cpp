// Energy rune predictor implementation (2026) - 重构版

#include "buff_predictor.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "plugin/debug/logger.hpp"

namespace autobuff::predictor {

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

    // 以第一个亮槽为锚点，用环形偏移排序，避免 0/2π 边界跳变
    double anchor = lit_slots[0].angle;
    auto ccw_offset = [anchor](double angle) {
        double d = angle - anchor;
        d = std::fmod(d, 2.0 * M_PI);
        if (d < 0.0) d += 2.0 * M_PI;
        return d;
    };

    autobuff::RotateDir dir = snap.direction;

    if (dir == autobuff::RotateDir::CW) {
        // 顺时针: 环形偏移从大到小 (等价于 CW 偏移从小到大)
        std::sort(lit_slots.begin(), lit_slots.end(),
                  [&](const SlotAngle& a, const SlotAngle& b) {
                      return ccw_offset(a.angle) > ccw_offset(b.angle);
                  });
    } else {
        // 逆时针或未知: 环形偏移从小到大
        std::sort(lit_slots.begin(), lit_slots.end(),
                  [&](const SlotAngle& a, const SlotAngle& b) {
                      return ccw_offset(a.angle) < ccw_offset(b.angle);
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
    // 7. Group PnP: 估计 rune 平面与中心 (委托 RuneObserver)
    // ============================================================
    auto obs = observer_.observe(det);

    if (!obs.valid) {
        snap.valid = false;
        return snap;
    }

    snap.center_cam = obs.center_cam;
    snap.center_world = obs.center_world;
    snap.normal_cam = obs.normal_cam;

    // 填充 2D + 3D 槽位信息
    for (int i = 0; i < NUM_SLOTS; ++i) {
        if (!det.targets[i].valid) continue;

        snap.slots[i].valid = true;
        snap.slots[i].is_lit = debounced.slots[i].is_lit;
        snap.slots[i].confidence = det.targets[i].confidence;
        snap.slots[i].center_px = det.targets[i].center;
        snap.slots[i].angle = det.targets[i].angle;

        if (obs.slots[i].valid) {
            snap.slots[i].pos_cam = obs.slots[i].pos_cam;
            snap.slots[i].vec_cam = obs.slots[i].vec_cam;
        }
    }

    snap.valid = true;

    // ============================================================
    // 8. 逆时针排名 (双车协同)
    // ============================================================
    build_ccw_rank(snap);

    return snap;
}

}  // namespace autobuff::predictor
