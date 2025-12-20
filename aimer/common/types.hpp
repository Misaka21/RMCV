//
// Created by nuc11 on 2025/10/5.
// 公共类型定义 - 消息传递与状态结构
//

#ifndef AIMER_COMMON_TYPES_HPP
#define AIMER_COMMON_TYPES_HPP

#include <chrono>
#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/mat.hpp>

namespace aimer {

using TimePoint = std::chrono::steady_clock::time_point;

// ============================================================================
// 1. 机器人状态 (RobotState)
// ============================================================================

/**
 * @brief 机器人状态 - 包含所有下位机数据
 *
 * 设计原则：
 * - 从 SerialReceiveData 构建，但不依赖其头文件
 * - 使用四元数而非欧拉角
 * - 时间戳使用本机采集时间
 */
struct RobotState {
    // IMU姿态 (云台坐标系 → 世界坐标系)
    Eigen::Quaterniond q_imu = Eigen::Quaterniond::Identity();

    // 云台速度 (云台坐标系下的vx, vy)
    Eigen::Vector2d velocity = Eigen::Vector2d::Zero();

    // 弹速 (m/s)
    float bullet_speed = 15.0f;

    // 敌方颜色 (0=未知, 1=红, 2=蓝)
    uint8_t enemy_color = 0;

    // 自瞄模式 (0=关闭, 1=自瞄, 2=小符, 3=大符)
    uint8_t aim_mode = 0;

    // 是否允许射击
    bool allow_fire = false;

    // 本机采集时间戳
    TimePoint timestamp = {};

    RobotState() = default;

    /**
     * @brief 从欧拉角构建 (ZYX顺序)
     * @param yaw 偏航角 (度)
     * @param pitch 俯仰角 (度)
     * @param roll 横滚角 (度)
     */
    static Eigen::Quaterniond euler_to_quaternion(float yaw, float pitch, float roll) {
        constexpr double deg2rad = M_PI / 180.0;
        double cy = std::cos(yaw * deg2rad * 0.5);
        double sy = std::sin(yaw * deg2rad * 0.5);
        double cp = std::cos(pitch * deg2rad * 0.5);
        double sp = std::sin(pitch * deg2rad * 0.5);
        double cr = std::cos(roll * deg2rad * 0.5);
        double sr = std::sin(roll * deg2rad * 0.5);

        Eigen::Quaterniond q;
        q.w() = cy * cp * cr + sy * sp * sr;
        q.x() = cy * cp * sr - sy * sp * cr;
        q.y() = cy * sp * cr + sy * cp * sr;
        q.z() = sy * cp * cr - cy * sp * sr;
        return q;
    }
};

// ============================================================================
// 2. 同步帧 (SyncFrame)
// ============================================================================

/**
 * @brief 同步帧 - 相机图像 + 机器人状态
 *
 * 由硬件层生成，传递给检测器
 * cv::Mat 使用浅拷贝，无额外开销
 */
struct SyncFrame {
    cv::Mat image;
    int frame_id = 0;
    RobotState state;

    SyncFrame() = default;
    SyncFrame(cv::Mat img, int id, const RobotState& s)
        : image(std::move(img)), frame_id(id), state(s) {}
};

// ============================================================================
// 3. 装甲板信息 (Armor)
// ============================================================================

/**
 * @brief 装甲板检测结果
 */
struct Armor {
    // 装甲板四角点 (图像坐标，左上顺时针)
    std::array<cv::Point2f, 4> corners;

    // 装甲板中心 (图像坐标)
    cv::Point2f center;

    // 装甲板ID (0-8: 对应数字，负数表示未识别)
    int id = -1;

    // 置信度 [0, 1]
    float confidence = 0.0f;

    // 装甲板类型 (0=小装甲板, 1=大装甲板)
    int type = 0;

    // PnP解算结果 (相机坐标系)
    Eigen::Vector3d position_camera = Eigen::Vector3d::Zero();
    bool pnp_valid = false;

    Armor() = default;
};

// ============================================================================
// 4. 检测结果 (DetectionResult)
// ============================================================================

/**
 * @brief 检测结果 - 传递给预测器
 *
 * 不再包含原始图像，只保留必要的状态信息
 */
struct DetectionResult {
    std::vector<Armor> armors;
    int frame_id = 0;
    RobotState state;

    DetectionResult() = default;
    DetectionResult(std::vector<Armor> a, int id, const RobotState& s)
        : armors(std::move(a)), frame_id(id), state(s) {}

    bool empty() const { return armors.empty(); }
    size_t size() const { return armors.size(); }
};

// ============================================================================
// 5. 预测结果 (PredictResult)
// ============================================================================

/**
 * @brief 预测结果 - 传递给弹道解算
 */
struct PredictResult {
    // 预测目标位置 (世界坐标系)
    Eigen::Vector3d target_world = Eigen::Vector3d::Zero();

    // 预测目标速度 (世界坐标系)
    Eigen::Vector3d target_vel = Eigen::Vector3d::Zero();

    // 目标ID
    int target_id = -1;

    // 是否有有效目标
    bool valid = false;

    int frame_id = 0;
    RobotState state;

    PredictResult() = default;
};

// ============================================================================
// 6. 发射指令 (FireCommand)
// ============================================================================

/**
 * @brief 发射指令 - 发送给下位机
 */
struct FireCommand {
    float yaw = 0.0f;    // 目标偏航角 (rad)
    float pitch = 0.0f;  // 目标俯仰角 (rad)
    float distance = 0.0f;  // 目标距离 (m)
    int target_id = 0;   // 目标ID
    bool fire = false;   // 是否开火

    FireCommand() = default;
    FireCommand(float y, float p, float d, int id, bool f)
        : yaw(y), pitch(p), distance(d), target_id(id), fire(f) {}
};

} // namespace aimer

#endif // AIMER_COMMON_TYPES_HPP
