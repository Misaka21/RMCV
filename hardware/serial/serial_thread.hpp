//
// Created by 霍睿 on 25-3-2.
//

#ifndef SERIAL_THREAD_HPP
#define SERIAL_THREAD_HPP

// C++ system headers
#include <string>
#include <queue>

// Third-party library headers

// Project headers
#include "transceiver_manager.hpp"
#include "fixed_packet.hpp"

namespace serial {

// 视觉数据结构体（示例，可根据实际需求修改）
struct VisionData_t {
    uint8_t  cmd_id;      // 数据类型，例如 0x01 = 自瞄数据
    float    yaw;         // 目标偏航角（°）
    float    pitch;       // 目标俯仰角（°）
    float    distance;    // 与目标距离（m）
    uint8_t  target_id;   // 打击对象ID（0~7：对应装甲板 or 英雄/工程）
    uint8_t  is_found;    // 是否发现目标 0=无 1=发现

    VisionData_t() : cmd_id(0x01), yaw(0.0f), pitch(0.0f), distance(0.0f), target_id(0), is_found(0) {}
};

// 接收到的串口数据结构体 - 从电控接收
struct SerialReceiveData {
    // IMU 姿态数据
    float yaw;            // 偏航角 (°)
    float pitch;          // 俯仰角 (°)
    float roll;           // 横滚角 (°)

    // 机器人状态
    uint8_t robot_id;     // 机器人ID (1-7红方, 101-107蓝方)
    uint8_t enemy_color;  // 敌方颜色 (0=未知, 1=红, 2=蓝)

    // 射击参数
    float bullet_speed;   // 弹速 (m/s)

    // 模式控制
    uint8_t aim_mode;     // 自瞄模式 (0=关闭, 1=自瞄, 2=小符, 3=大符)
    bool allow_fire;      // 是否允许射击
    bool aiming_lock;     // 预瞄锁定 (右键按下=true, 释放=false)

    // 时间戳 (上位机接收时刻，微秒)
    int64_t recv_time_us = 0;

    SerialReceiveData()
        : yaw(0.0f), pitch(0.0f), roll(0.0f)
        , robot_id(0), enemy_color(0)
        , bullet_speed(15.0f)
        , aim_mode(0), allow_fire(false), aiming_lock(false)
        , recv_time_us(0) {}
};

// 接收数据队列类型别名 - 使用STL queue
using ReceiveQueue = std::queue<SerialReceiveData>;

/**
 * @brief 串口发送线程主函数（内部使用）
 * @param transceiver 共享的TransceiverManager实例
 */
void serial_sender_run(std::shared_ptr<TransceiverManager<16>> transceiver);

/**
 * @brief 串口接收线程主函数（内部使用）
 * @param transceiver 共享的TransceiverManager实例
 */
void serial_receiver_run(std::shared_ptr<TransceiverManager<16>> transceiver);

/**
 * @brief 启动串口通信（同时启动发送和接收线程，共享串口实例）
 * @param port_path 串口设备路径
 * @param baud_rate 波特率
 *
 * 该函数会创建一个串口实例，并同时启动发送和接收线程
 * 确保发送和接收共享同一个串口设备
 */
void start_serial_communication(const std::string& port_path = "/dev/ttyUSB0", int baud_rate = 115200);

/**
 * @brief 串口工具类，用于包转换和底层操作
 */
class SerialUtils {
public:
    using PacketType = FixedPacket<16>;

    /**
     * @brief 将视觉数据转换为数据包
     * @param cmd 视觉数据
     * @param packet 输出数据包
     * @return true 转换成功，false 转换失败
     */
    static bool vision_data_to_packet(const VisionData_t& cmd, PacketType& packet);

    /**
     * @brief 将接收到的数据包转换为结构体
     * @param packet 数据包
     * @param data 输出数据结构体
     * @return true 转换成功，false 转换失败
     */
    static bool packet_to_receive_data(const PacketType& packet, SerialReceiveData& data);
};

} // namespace serial

#endif //SERIAL_THREAD_HPP