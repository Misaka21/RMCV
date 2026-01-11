/**
 * @file autoaim_target_adapter.hpp
 * @brief AutoAim 目标适配器 - 将 VehicleState/ArmorState 转换为 TargetInterface
 *
 * 适配器模式: 让火控模块独立于 predictor 类型
 */

#ifndef __AIMER_AUTO_AIM_FIRE_CONTROL_AUTOAIM_TARGET_ADAPTER_HPP__
#define __AIMER_AUTO_AIM_FIRE_CONTROL_AUTOAIM_TARGET_ADAPTER_HPP__

#include <memory>

#include "aimer/fire_control/interface/target_interface.hpp"
#include "aimer/fire_control/interface/target_snapshot.hpp"
#include "aimer/auto_aim/predictor/types.hpp"
#include "aimer/common/robot_state.hpp"

namespace autoaim::fire_control {

/**
 * @brief 装甲板目标适配器
 *
 * 将 ArmorState 适配为 TargetInterface
 * 用于非陀螺目标的直接打击
 */
class ArmorTargetAdapter : public ::fire_control::TargetInterface {
public:
    ArmorTargetAdapter(const predictor::ArmorState* armor, int target_id)
        : armor_(armor), target_id_(target_id) {}

    bool is_valid() const override {
        return armor_ != nullptr && armor_->visible;
    }

    int target_id() const override { return target_id_; }

    Eigen::Vector3d position() const override {
        return armor_ ? armor_->position : Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d velocity() const override {
        return armor_ ? armor_->velocity : Eigen::Vector3d::Zero();
    }

    ::fire_control::TargetSize size() const override {
        if (!armor_) return {0.135, 0.055};  // 默认小装甲板
        return {armor_->width(), armor_->height()};
    }

    double orientation_angle() const override {
        return armor_ ? armor_->yaw : 0;
    }

    double confidence() const override {
        return armor_ ? armor_->score : 0;
    }

    double z_to_view_angle() const override {
        return armor_ ? armor_->z_to_v : 0;
    }

    // 非旋转目标
    bool is_rotating() const override { return false; }

private:
    const predictor::ArmorState* armor_;
    int target_id_;
};

/**
 * @brief 车辆目标适配器 (陀螺目标)
 *
 * 将 VehicleState 适配为 TargetInterface
 * 用于陀螺目标的旋转中心打击
 */
class VehicleTargetAdapter : public ::fire_control::TargetInterface {
public:
    explicit VehicleTargetAdapter(const predictor::VehicleState* vehicle)
        : vehicle_(vehicle) {}

    bool is_valid() const override {
        return vehicle_ != nullptr && vehicle_->valid;
    }

    int target_id() const override {
        return vehicle_ ? vehicle_->target_id : -1;
    }

    Eigen::Vector3d position() const override {
        return vehicle_ ? vehicle_->center : Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d velocity() const override {
        return vehicle_ ? vehicle_->velocity : Eigen::Vector3d::Zero();
    }

    ::fire_control::TargetSize size() const override {
        // 陀螺目标使用较大的等效尺寸
        if (!vehicle_) return {0.2, 0.1};
        const auto* armor = vehicle_->get_recommended_armor();
        if (armor) return {armor->width(), armor->height()};
        return {0.2, 0.1};
    }

    double orientation_angle() const override {
        return vehicle_ ? vehicle_->spin.phase : 0;
    }

    double confidence() const override {
        return vehicle_ ? vehicle_->confidence : 0;
    }

    // 陀螺目标的旋转信息
    bool is_rotating() const override {
        return vehicle_ && vehicle_->spin.active;
    }

    bool is_high_speed_rotating() const override {
        return vehicle_ && vehicle_->spin.active &&
               vehicle_->spin.level == predictor::SpinLevel::HIGH;
    }

    double angular_velocity() const override {
        return vehicle_ ? vehicle_->spin.omega : 0;
    }

    double phase() const override {
        return vehicle_ ? vehicle_->spin.phase : 0;
    }

    Eigen::Vector3d rotation_center() const override {
        return position();  // 旋转中心就是 center
    }

    // 多子目标扩展 (装甲板)
    int sub_target_count() const override {
        return vehicle_ ? vehicle_->armor_count : 0;
    }

    Eigen::Vector3d predict_sub_target_position(int index, double dt) const override {
        if (!vehicle_ || index < 0 || index >= vehicle_->armor_count) {
            return predict_center(dt);
        }
        return vehicle_->predict_armor_position(index, dt);
    }

    Eigen::Vector3d predict_center(double dt) const override {
        return vehicle_ ? vehicle_->predict_center(dt) : Eigen::Vector3d::Zero();
    }

    // 获取特定装甲板位置 (便捷方法，同 predict_sub_target_position)
    Eigen::Vector3d get_armor_position(int armor_idx, double dt = 0) const {
        if (!vehicle_ || armor_idx < 0 || armor_idx >= vehicle_->armor_count) {
            return position();
        }
        return vehicle_->predict_armor_position(armor_idx, dt);
    }

    // 获取推荐装甲板
    int get_recommended_armor_idx() const {
        return vehicle_ ? vehicle_->recommended_armor_idx : -1;
    }

    // 装甲板数量
    int armor_count() const {
        return vehicle_ ? vehicle_->armor_count : 0;
    }

    // 获取原始 VehicleState (供需要详细信息的组件使用)
    const predictor::VehicleState* raw() const { return vehicle_; }

private:
    const predictor::VehicleState* vehicle_;
};

/**
 * @brief 战场快照适配器
 *
 * 将 BattlefieldSnapshot 适配为 TargetSnapshotInterface
 */
class BattlefieldSnapshotAdapter : public ::fire_control::TargetSnapshotInterface {
public:
    explicit BattlefieldSnapshotAdapter(const predictor::BattlefieldSnapshot* snapshot)
        : snapshot_(snapshot)
        , primary_adapter_(nullptr)
    {
        update_self_state();
        update_primary_adapter();
    }

    double timestamp() const override {
        return snapshot_ ? snapshot_->timestamp : 0;
    }

    double predict_timestamp() const override {
        return snapshot_ ? snapshot_->predict_timestamp : 0;
    }

    const ::fire_control::SelfState& self_state() const override {
        return self_state_;
    }

    const ::fire_control::TargetInterface* primary_target() const override {
        return primary_adapter_.get();
    }

    int target_count() const override {
        if (!snapshot_) return 0;
        int count = 0;
        for (int i = 0; i < predictor::MAX_TARGETS; ++i) {
            if (snapshot_->is_valid(i)) ++count;
        }
        return count;
    }

    int frame_id() const override {
        return snapshot_ ? snapshot_->frame_id : 0;
    }

    // 获取原始快照 (供需要详细信息的组件使用)
    const predictor::BattlefieldSnapshot* raw() const { return snapshot_; }

private:
    void update_self_state() {
        if (!snapshot_) return;

        const auto& rs = snapshot_->self_state;
        self_state_.orientation = rs.q_imu;
        self_state_.bullet_speed = rs.bullet_speed;
        self_state_.velocity = rs.velocity;
        self_state_.update_from_quaternion();
    }

    void update_primary_adapter() {
        if (!snapshot_ || snapshot_->primary_target_id < 0) {
            primary_adapter_.reset();
            return;
        }

        const auto* primary = snapshot_->get_primary();
        if (primary) {
            primary_adapter_ = std::make_unique<VehicleTargetAdapter>(primary);
        }
    }

    const predictor::BattlefieldSnapshot* snapshot_;
    ::fire_control::SelfState self_state_;
    std::unique_ptr<VehicleTargetAdapter> primary_adapter_;
};

}  // namespace autoaim::fire_control

#endif  // __AIMER_AUTO_AIM_FIRE_CONTROL_AUTOAIM_TARGET_ADAPTER_HPP__
