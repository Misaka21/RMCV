// PnP 输出的 3D 观测结果 (observer → predictor 内部边界)

#ifndef AIMER_AUTOBUFF_PREDICTOR_OBSERVER_RUNE_OBSERVATION_HPP
#define AIMER_AUTOBUFF_PREDICTOR_OBSERVER_RUNE_OBSERVATION_HPP

#include <array>

#include <Eigen/Core>

#include "aimer/auto_buff/common/types.hpp"

namespace autobuff::predictor {

struct RuneSlotObservation {
    bool valid = false;
    Eigen::Vector3d pos_cam = Eigen::Vector3d::Zero();
    Eigen::Vector3d pos_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d vec_cam = Eigen::Vector3d::Zero();  // center→slot 向量 (旋转预测用)
};

struct RuneObservation {
    bool valid = false;
    Eigen::Vector3d center_cam = Eigen::Vector3d::Zero();
    Eigen::Vector3d center_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal_cam = Eigen::Vector3d(0, 0, 1);
    std::array<RuneSlotObservation, NUM_SLOTS> slots{};
};

}  // namespace autobuff::predictor

#endif  // AIMER_AUTOBUFF_PREDICTOR_OBSERVER_RUNE_OBSERVATION_HPP
