// auto_buff predictor types (2026)
//
// Dataflow:
//   BuffDetectionResult ("buff_detections") -> BuffSnapshot ("buff_snapshot") -> fire_control

#ifndef AIMER_AUTOBUFF_PREDICTOR_TYPES_HPP
#define AIMER_AUTOBUFF_PREDICTOR_TYPES_HPP

#include <array>
#include <cmath>
#include <cstdint>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include "aimer/auto_buff/common/types.hpp"
#include "aimer/common/robot_state.hpp"

namespace autobuff::predictor {

enum class SpeedModel : uint8_t {
    UNKNOWN = 0,
    CONST_OMEGA = 1,      // 恒速 (小符 / 大符非激活)
    LARGE_SINE_LSM = 2,   // 正弦拟合 (大符激活)
};

struct LargeSineParam {
    bool valid = false;
    int dir = 1;              // +1 CCW, -1 CW
    double start_time = 0.0;  // t=0 绝对时间戳

    double a = 0.90;          // [0.780, 1.045]
    double w = 1.94;          // [1.884, 2.000]
    double tau = 0.0;         // 时间偏移 [-0.5, 0.5]
    double phi0 = 0.0;        // 初始相位偏移

    double residual_rms = 1e9;
    int sample_count = 0;

    double b() const { return 2.090 - a; }

    // 累积角度: ∫spd(t)dt = -(a/w)*cos(w*(t+tau)) + b*t
    double phi(double t_rel) const {
        return static_cast<double>(dir)
             * (-(a / w) * std::cos(w * (t_rel + tau)) + b() * t_rel)
             + phi0;
    }

    double delta(double t_rel, double dt) const {
        return phi(t_rel + dt) - phi(t_rel);
    }
};

// 运动估计结果 (模型输出, 供火控插值)
struct MotionEstimate {
    SpeedModel model = SpeedModel::UNKNOWN;
    double omega_signed = 0.0;   // CONST_OMEGA 模式: 带符号角速度
    LargeSineParam large{};      // LARGE_SINE_LSM 模式
    double confidence = 0.0;

    // 统一的 delta_theta 计算 (火控 500Hz 插值用)
    double delta_theta(double t_abs, double dt) const {
        if (model == SpeedModel::LARGE_SINE_LSM && large.valid) {
            double t_rel = t_abs - large.start_time;
            return large.delta(t_rel, dt);
        }
        return omega_signed * dt;
    }
};

struct RuneSlotState {
    bool valid = false;
    bool is_lit = false;
    float confidence = 0.f;

    cv::Point2f center_px{};
    double angle = 0.0;  // rad (math coord, x right, y up), relative to R center

    // 3D point at image timestamp
    Eigen::Vector3d pos_cam = Eigen::Vector3d::Zero();

    // Vector from center -> slot point in camera frame (for rotation prediction)
    Eigen::Vector3d vec_cam = Eigen::Vector3d::Zero();
};

struct BuffSnapshot {
    bool valid = false;

    int frame_id = 0;
    double timestamp = 0.0;        // image timestamp (s)
    double predict_timestamp = 0.0;// predictor finish timestamp (s)

    aimer::RobotState self_state;

    // 模式与方向
    autobuff::BuffMode mode = autobuff::BuffMode::UNKNOWN;
    autobuff::RotateDir direction = autobuff::RotateDir::UNKNOWN;

    // 运动估计 (统一接口)
    MotionEstimate motion;

    // Rotation center / plane in camera/world (at image time)
    Eigen::Vector3d center_cam = Eigen::Vector3d::Zero();
    Eigen::Vector3d center_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal_cam = Eigen::Vector3d(0, 0, 1);

    // Slots
    std::array<RuneSlotState, NUM_SLOTS> slots{};
    uint8_t lit_mask = 0;
    int lit_count = 0;
    int recommended_slot = -1;

    // 双车协同: 逆时针排序 (rank[0]=逆时针第1个lit, rank[1]=第2个lit, ...)
    std::array<int, NUM_SLOTS> ccw_lit_rank{{-1, -1, -1, -1, -1}};
    int ranked_count = 0;

    // === 辅助方法 ===

    bool has_slot(int slot_id) const {
        return slot_id >= 0 && slot_id < NUM_SLOTS && slots[slot_id].valid;
    }

    bool is_lit(int slot_id) const {
        return has_slot(slot_id) && slots[slot_id].is_lit;
    }

    // 旋转预测 (火控 500Hz 插值用)
    Eigen::Vector3d predict_slot_cam(int slot_id, double dt) const {
        if (!valid || !has_slot(slot_id)) return Eigen::Vector3d::Zero();

        double delta = motion.delta_theta(timestamp, dt);

        Eigen::AngleAxisd aa(delta, normal_cam.normalized());
        Eigen::Vector3d v2 = aa.toRotationMatrix() * slots[slot_id].vec_cam;
        return center_cam + v2;
    }

};

}  // namespace autobuff::predictor

#endif  // AIMER_AUTOBUFF_PREDICTOR_TYPES_HPP
