/**
 * @file armor_table.hpp
 * @brief 装甲板观测表
 *
 * 每帧检测后，所有装甲板观测存入此表，
 * EnemyState 通过 target_id 查找属于自己的装甲板
 */

#ifndef __AIMER_AUTO_AIM_PREDICTOR_ENEMY_STATE_ARMOR_TABLE_HPP__
#define __AIMER_AUTO_AIM_PREDICTOR_ENEMY_STATE_ARMOR_TABLE_HPP__

#include <unordered_map>
#include <vector>

#include "aimer/auto_aim/predictor/types.hpp"

namespace autoaim::predictor {

/**
 * @brief 装甲板观测表
 *
 * 用法:
 *   // 每帧开始
 *   table.clear();
 *
 *   // PnP 解算后添加
 *   for (auto& det : detections) {
 *       auto obs = ArmorObservation::from_detection(...);
 *       table.add(obs);
 *   }
 *
 *   // EnemyState 查询
 *   enemy_state.update(table.get(target_id), timestamp);
 */
class ArmorObservationTable {
public:
    // 清空表 (每帧开始调用)
    void clear() {
        table_.clear();
        timestamp_ = 0;
        frame_id_ = 0;
    }

    // 设置帧信息
    void set_frame(double timestamp, int frame_id) {
        timestamp_ = timestamp;
        frame_id_ = frame_id;
    }

    // 添加观测 (自动按 target_id 分组)
    void add(const ArmorObservation& obs) {
        if (obs.valid && obs.target_id > 0) {
            table_[obs.target_id].push_back(obs);
        }
    }

    // 批量添加
    void add(const std::vector<ArmorObservation>& observations) {
        for (const auto& obs : observations) {
            add(obs);
        }
    }

    // 查询某目标的所有观测
    const std::vector<ArmorObservation>& get(int target_id) const {
        static const std::vector<ArmorObservation> empty;
        auto it = table_.find(target_id);
        return (it != table_.end()) ? it->second : empty;
    }

    // 检查目标是否有观测
    bool has(int target_id) const {
        auto it = table_.find(target_id);
        return it != table_.end() && !it->second.empty();
    }

    // 获取所有有观测的目标 ID
    std::vector<int> get_target_ids() const {
        std::vector<int> ids;
        ids.reserve(table_.size());
        for (const auto& [id, _] : table_) {
            ids.push_back(id);
        }
        return ids;
    }

    // 获取观测位掩码 (bit i = target i 是否有观测)
    uint16_t get_detection_mask() const {
        uint16_t mask = 0;
        for (const auto& [id, _] : table_) {
            if (id >= 0 && id < 16) {
                mask |= (1 << id);
            }
        }
        return mask;
    }

    // 总观测数
    size_t total_count() const {
        size_t count = 0;
        for (const auto& [_, armors] : table_) {
            count += armors.size();
        }
        return count;
    }

    // 目标数
    size_t target_count() const { return table_.size(); }

    // 时间戳
    double timestamp() const { return timestamp_; }
    int frame_id() const { return frame_id_; }

    // 遍历所有目标
    template<typename Func>
    void for_each_target(Func&& func) const {
        for (const auto& [id, armors] : table_) {
            func(id, armors);
        }
    }

private:
    std::unordered_map<int, std::vector<ArmorObservation>> table_;
    double timestamp_ = 0;
    int frame_id_ = 0;
};

}  // namespace autoaim::predictor

#endif  // __AIMER_AUTO_AIM_PREDICTOR_ENEMY_STATE_ARMOR_TABLE_HPP__
