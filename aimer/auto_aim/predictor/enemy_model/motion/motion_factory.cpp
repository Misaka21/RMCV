/**
 * @file motion_factory.cpp
 * @brief 运动模型工厂实现
 */

#include "motion_factory.hpp"

#include "spin_motion.hpp"
#include "lmtd_motion.hpp"
#include "sp_motion.hpp"

namespace autoaim::predictor {

std::unique_ptr<MotionInterface> create_motion(MotionType type, int armor_num) {
    switch (type) {
        case MotionType::SP:
            return std::make_unique<SpMotion>(armor_num);
        case MotionType::LMTD:
            return std::make_unique<LmtdMotion>(armor_num);
        case MotionType::SPIN:
        default:
            return std::make_unique<SpinMotion>(armor_num);
    }
}

std::unique_ptr<MotionInterface> create_motion(const std::string& type_str, int armor_num) {
    return create_motion(motion_type_from_string(type_str), armor_num);
}

MotionType motion_type_from_string(const std::string& type_str) {
    if (type_str == "sp") {
        return MotionType::SP;
    } else if (type_str == "lmtd") {
        return MotionType::LMTD;
    } else {
        return MotionType::SPIN;
    }
}

const char* motion_type_to_string(MotionType type) {
    switch (type) {
        case MotionType::SP:
            return "sp";
        case MotionType::LMTD:
            return "lmtd";
        case MotionType::SPIN:
        default:
            return "spin";
    }
}

}  // namespace autoaim::predictor
