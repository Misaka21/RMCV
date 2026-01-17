/**
 * @file armor_identifier.cpp
 * @brief 装甲板 ID 分配器实现
 */

#include "armor_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <deque>

namespace autoaim::predictor {

// ============================================================================
// LightThread
// ============================================================================

LightThread::LightThread(int id, const ArmorObservation& obs, double timestamp, int frame)
    : id_(id), obs_(obs), frame_(frame), last_update_time_(timestamp) {}

void LightThread::update(const ArmorObservation& obs, double timestamp, int frame) {
    obs_ = obs;
    frame_ = frame;
    last_update_time_ = timestamp;
}

bool LightThread::alive(double current_time) const {
    return (current_time - last_update_time_) <= LIGHT_LIFE;
}

double LightThread::get_cost(const ArmorObservation& obs) const {
    double cost = 0;

    // number 不匹配惩罚
    if (obs.target_id != obs_.target_id) {
        cost += 1.1;
    }

    // 位置距离代价
    // 消除深度差异的影响：将两个位置投影到相同距离
    double in_dist = obs_.distance();
    double out_dist = obs.distance();

    Eigen::Vector3d in_pos = obs_.pos;
    Eigen::Vector3d out_pos = obs.pos;

    if (out_dist > 0.1 && in_dist > 0.1) {
        out_pos = out_pos * (in_dist / out_dist);
    }

    // 装甲板对角线估计
    double diag = (obs_.type == ArmorType::LARGE) ? 0.5 : 0.3;

    // 代价 = 距离 / (对角线/2) * 0.5
    cost += (in_pos - out_pos).norm() / (diag / 2.0) * 0.5;

    return cost;
}

bool LightThread::collide(const LightThread& other) const {
    return this->get_cost(other.observation()) + other.get_cost(this->observation()) < 2.0;
}

ArmorData LightThread::get_armor_data() const {
    ArmorData data;
    data.id = id_;
    data.observation = obs_;
    return data;
}

// ============================================================================
// ArmorIdentifier
// ============================================================================

void ArmorIdentifier::update(
    const std::vector<ArmorObservation>& observations,
    double timestamp,
    int frame
) {
    current_frame_ = frame;

    // 1. 准备候选列表
    std::deque<ArmorObservation> candidates(observations.begin(), observations.end());

    // 2. 现有线程尝试匹配 (贪心算法)
    for (auto& [id, thread] : threads_) {
        auto found = candidates.end();
        double min_cost = MATCH_COST_THRESHOLD;

        for (auto it = candidates.begin(); it != candidates.end(); ++it) {
            double cost = thread.get_cost(*it);
            if (cost < min_cost) {
                min_cost = cost;
                found = it;
            }
        }

        if (found != candidates.end()) {
            thread.update(*found, timestamp, frame);
            candidates.erase(found);
        }
    }

    // 3. 为未匹配的观测创建新线程
    for (const auto& obs : candidates) {
        if (obs.valid) {
            threads_.emplace(
                next_id_,
                LightThread(next_id_, obs, timestamp, frame)
            );
            ++next_id_;
        }
    }

    // 4. 清理超时线程
    for (auto it = threads_.begin(); it != threads_.end();) {
        if (!it->second.alive(timestamp)) {
            it = threads_.erase(it);
        } else {
            ++it;
        }
    }

    // 5. 清理碰撞/过多的线程
    std::map<int, int> count_by_target;
    for (const auto& [id, thread] : threads_) {
        count_by_target[thread.target_id()]++;
    }

    for (auto it = threads_.begin(); it != threads_.end();) {
        const auto& thread = it->second;

        // 非活跃线程才检查碰撞和数量限制
        if (!thread.active(frame)) {
            bool has_collision = false;
            for (const auto& [other_id, other_thread] : threads_) {
                if (it->first == other_id) continue;
                if (thread.collide(other_thread)) {
                    has_collision = true;
                    break;
                }
            }

            int target_id = thread.target_id();
            bool too_many = count_by_target[target_id] > MAX_THREADS_PER_TARGET;

            if (has_collision || too_many) {
                count_by_target[target_id]--;
                it = threads_.erase(it);
                continue;
            }
        }
        ++it;
    }
}

std::vector<ArmorData> ArmorIdentifier::get_active_armors(int frame) const {
    std::vector<ArmorData> result;
    for (const auto& [id, thread] : threads_) {
        if (thread.active(frame)) {
            result.push_back(thread.get_armor_data());
        }
    }
    return result;
}

std::vector<ArmorData> ArmorIdentifier::get_all_armors() const {
    std::vector<ArmorData> result;
    for (const auto& [id, thread] : threads_) {
        result.push_back(thread.get_armor_data());
    }
    return result;
}

void ArmorIdentifier::reset() {
    threads_.clear();
    next_id_ = 1;
}

}  // namespace autoaim::predictor
