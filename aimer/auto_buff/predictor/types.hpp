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
#include "aimer/common/transformer/transformer.hpp"

namespace autobuff::predictor {

enum class SpeedModel : uint8_t {
    UNKNOWN = 0,
    CONSTANT = 1,   // omega is constant
    BIG_SINE = 2    // omega(t)=a*sin(w*t)+b, b=2.090-a (2026 large active)
};

struct BigSineModel {
    bool valid = false;
    int dir = 1;            // +1 or -1
    double start_time = 0;  // absolute timestamp when t=0

    double a = 0.90;        // [0.780, 1.045]
    double w = 1.94;        // [1.884, 2.000]
    double t_shift = 0.0;   // time offset (s), typically within +-0.5s
    double c = 0.0;         // phase offset

    double b() const { return 2.090 - a; }

    double phi(double t) const {
        // Integral of spd(t)=a*sin(w*(t+t_shift))+b => phi(t)=-(a/w)*cos(w*(t+t_shift))+b*t + const
        return static_cast<double>(dir)
             * (-(a / w) * std::cos(w * (t + t_shift)) + b() * t)
             + c;
    }

    double delta(double t, double dt) const {
        return phi(t + dt) - phi(t);
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
    Eigen::Vector3d pos_world = Eigen::Vector3d::Zero();

    // Vector from center -> slot point in camera frame (for rotation prediction)
    Eigen::Vector3d vec_cam = Eigen::Vector3d::Zero();
};

struct BuffSnapshot {
    bool valid = false;

    int frame_id = 0;
    double timestamp = 0.0;        // image timestamp (s)
    double predict_timestamp = 0.0;// predictor finish timestamp (s)

    aimer::RobotState self_state;

    // Rotation center / plane in camera/world (at image time)
    Eigen::Vector3d center_cam = Eigen::Vector3d::Zero();
    Eigen::Vector3d center_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal_cam = Eigen::Vector3d(0, 0, 1);  // normalized, facing camera (+z) if possible

    // Slots
    std::array<RuneSlotState, NUM_SLOTS> slots{};
    uint8_t lit_mask = 0;
    int lit_count = 0;
    int recommended_slot = -1;

    // Speed model (global)
    SpeedModel model = SpeedModel::UNKNOWN;
    double omega = 0.0;     // for CONSTANT (signed)
    BigSineModel big;       // for BIG_SINE

    bool has_slot(int slot_id) const {
        return slot_id >= 0 && slot_id < NUM_SLOTS && slots[slot_id].valid;
    }

    bool is_lit(int slot_id) const {
        return has_slot(slot_id) && slots[slot_id].is_lit;
    }

    Eigen::Vector3d predict_slot_cam(int slot_id, double dt) const {
        if (!valid || !has_slot(slot_id)) return Eigen::Vector3d::Zero();

        double delta = 0.0;
        if (model == SpeedModel::BIG_SINE && big.valid) {
            double t = timestamp - big.start_time;
            delta = big.delta(t, dt);
        } else if (model == SpeedModel::CONSTANT) {
            delta = omega * dt;
        } else {
            delta = omega * dt;
        }

        Eigen::AngleAxisd aa(delta, normal_cam.normalized());
        Eigen::Vector3d v2 = aa.toRotationMatrix() * slots[slot_id].vec_cam;
        return center_cam + v2;
    }

    Eigen::Vector3d predict_slot_world(int slot_id, double dt) const {
        Eigen::Vector3d p_cam = predict_slot_cam(slot_id, dt);
        if (p_cam.isZero(0)) return Eigen::Vector3d::Zero();
        return aimer::tf::cam_to_world(p_cam, self_state.q_imu);
    }
};

}  // namespace autobuff::predictor

#endif  // AIMER_AUTOBUFF_PREDICTOR_TYPES_HPP
