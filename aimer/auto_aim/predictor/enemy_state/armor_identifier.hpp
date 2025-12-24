/**
 * @file armor_identifier.hpp
 * @brief 装甲板 ID 分配器 - 跨帧匹配和 ID 分配
 *
 * 参考: rm.cv.fans 的 LightManager
 *
 * 职责 (单一):
 * - 跨帧装甲板匹配
 * - 分配稳定的 ID
 * - 超时清理
 *
 * 不做:
 * - EKF 滤波 (交给 ArmorMotion)
 */

#ifndef __AIMER_AUTO_AIM_PREDICTOR_ENEMY_MODEL_ARMOR_IDENTIFIER_HPP__
#define __AIMER_AUTO_AIM_PREDICTOR_ENEMY_MODEL_ARMOR_IDENTIFIER_HPP__

#include <map>
#include <vector>

#include "aimer/auto_aim/predictor/types.hpp"

namespace autoaim::predictor {

/**
 * @brief 带稳定 ID 的装甲板数据
 *
 * ArmorIdentifier 的输出，ArmorMotion 的输入
 */
struct ArmorData {
    int id = -1;                    // 分配的稳定 ID (跨帧一致)
    ArmorObservation observation;   // 原始观测数据

    // 便捷访问
    int target_id() const { return observation.target_id; }
    const Eigen::Vector3d& pos() const { return observation.pos; }
    double distance() const { return observation.distance(); }
    double z_to_v() const { return observation.z_to_v; }
    ArmorType type() const { return observation.type; }
};

/**
 * @brief 单个装甲板跟踪线程
 *
 * 只负责 ID 维护和匹配，不做滤波
 */
class LightThread {
public:
    LightThread(int id, const ArmorObservation& obs, double timestamp, int frame);

    /**
     * @brief 更新观测
     */
    void update(const ArmorObservation& obs, double timestamp, int frame);

    /**
     * @brief 是否当前帧激活
     */
    bool active(int frame) const { return frame_ == frame; }

    /**
     * @brief 是否存活 (未超时)
     */
    bool alive(double current_time) const;

    /**
     * @brief 计算与某观测的匹配代价
     *
     * 代价计算:
     * - number 不匹配: +1.1
     * - 位置距离 / (装甲板对角线/2) * 0.5
     *
     * 代价 < 0.5 则认为匹配
     */
    double get_cost(const ArmorObservation& obs) const;

    /**
     * @brief 检查两线程是否碰撞
     */
    bool collide(const LightThread& other) const;

    /**
     * @brief 获取带 ID 的装甲板数据
     */
    ArmorData get_armor_data() const;

    // Getters
    int id() const { return id_; }
    int target_id() const { return obs_.target_id; }
    const ArmorObservation& observation() const { return obs_; }

private:
    int id_;                         // 分配的唯一 ID
    ArmorObservation obs_;           // 最近的观测
    int frame_ = 0;                  // 最后更新的帧号
    double last_update_time_ = 0;    // 最后更新时间

    // 参数
    static constexpr double LIGHT_LIFE = 0.100;  // 线程存活时间 (s)
};

/**
 * @brief 装甲板 ID 分配器
 *
 * 管理多个 LightThread，负责:
 * - 跨帧匹配
 * - ID 分配
 * - 超时/碰撞清理
 */
class ArmorIdentifier {
public:
    ArmorIdentifier() = default;

    /**
     * @brief 更新
     * @param observations 当前帧的观测列表
     * @param timestamp 时间戳
     * @param frame 帧号
     */
    void update(const std::vector<ArmorObservation>& observations, double timestamp, int frame);

    /**
     * @brief 获取当前帧激活的装甲板 (带 ID)
     */
    std::vector<ArmorData> get_active_armors(int frame) const;

    /**
     * @brief 获取所有存活的装甲板 (包括短暂丢失的)
     */
    std::vector<ArmorData> get_all_armors() const;

    /**
     * @brief 线程数量
     */
    size_t size() const { return threads_.size(); }

    /**
     * @brief 重置
     */
    void reset();

private:
    std::map<int, LightThread> threads_;  // id -> thread
    int next_id_ = 1;                     // 下一个分配的 ID
    int current_frame_ = 0;               // 当前帧号

    // 参数
    static constexpr double MATCH_COST_THRESHOLD = 0.5;
    static constexpr int MAX_THREADS_PER_TARGET = 3;
};

}  // namespace autoaim::predictor

#endif  // __AIMER_AUTO_AIM_PREDICTOR_ENEMY_MODEL_ARMOR_IDENTIFIER_HPP__
