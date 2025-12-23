//
// Simulator Node - ROS2模拟器接入模块
// 替代hardware模块，用于接入at_vision_simulator进行benchmark测试
//
// 订阅Topics (from at_vision_simulator):
//   /image_raw    - sensor_msgs/Image
//   /gimbal_pose  - geometry_msgs/PoseStamped (四元数)
//

#ifndef SIMULATOR_NODE_HPP
#define SIMULATOR_NODE_HPP

#include <string>

namespace simulator {

/**
 * @brief 模拟器配置
 */
struct SimulatorConfig {
    // ROS2 Topics
    std::string image_topic = "/image_raw";
    std::string gimbal_pose_topic = "/gimbal_pose";

    // 模拟串口数据
    uint8_t robot_id = 1;
    uint8_t enemy_color = 2;      // 1=红, 2=蓝
    float bullet_speed = 15.0f;
    uint8_t aim_mode = 1;
    bool allow_fire = true;
};

/**
 * @brief 启动模拟器节点 (阻塞)
 *
 * 替代 hardware::start_hardware_node()
 * 发布 SyncFrame 到 UMT channel "sync_frame"
 */
void start_simulator_node();

/**
 * @brief 从配置文件加载配置
 */
SimulatorConfig load_config(const std::string& filename = "simulator.toml");

} // namespace simulator

#endif // SIMULATOR_NODE_HPP
