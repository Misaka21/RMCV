//
// Created by 霍睿 on 25-3-2.
//

#ifndef SERIAL_THREAD_HPP
#define SERIAL_THREAD_HPP

// C++ system headers
#include <cstdint>
#include <string>

// Third-party library headers

// Project headers
#include "transceiver_manager.hpp"
#include "fixed_packet.hpp"

namespace serial {

// ============================================================================
// 数据结构
// ============================================================================

// TODO: RTT测量功能 (最简+序号方案)
// ============================================================================
// 目的: 实测串口延迟，替代火控里硬编码的 send_to_control = 0.003
//
// 协议改动:
//   1. 扩大包: FixedPacket<16> → FixedPacket<24> 或 32
//   2. 上行包增加: seq(2B) + timestamp_ms(4B)
//   3. 下行包增加: echo_seq(2B) + echo_tick_ms(4B)
//
// 原理:
//   上位机 --[seq, T1]--> MCU --[echo_seq, echo_T1]--> 上位机
//   RTT = recv_time - echo_T1
//   send_to_control = RTT / 2
//
// 电控配合:
//   - 收到上行包时保存 seq 和 timestamp_ms
//   - 下行包里原样回传 echo_seq 和 echo_tick_ms
//
// 实现步骤:
//   1. VisionData_t 加 uint16_t seq, uint32_t timestamp_ms
//   2. SerialReceiveData 加 uint16_t echo_seq, uint32_t echo_tick_ms
//   3. 新建 RttStats 类统计 RTT 均值/标准差/丢包率
//   4. LatencyEstimator::build() 用实测 RTT/2 替代 get_param("send_to_control")
// ============================================================================

// 视觉数据结构体 (视觉 → 电控, 32字节协议)
// 协议布局:
//   [0] head=0xff, [1] control, [2] shoot,
//   [3-6] yaw, [7-10] pitch, [11-28] reserved,
//   [29-30] crc16, [31] tail=0x0d
struct VisionData_t {
    uint8_t control;      // 控制标志: 1=控制
    uint8_t shoot;        // 射击标志: 1=射击
    float   yaw;          // 目标偏航角（弧度）
    float   pitch;        // 目标俯仰角（弧度）

    VisionData_t() : control(0), shoot(0), yaw(0.0f), pitch(0.0f) {}
};

// 接收到的串口数据结构体 (电控 → 视觉, 32字节协议)
// 协议布局:
//   [0] head=0xff, [1] mode, [2] aiming_lock,
//   [3-6] bullet_speed, [7-10] yaw, [11-14] pitch, [15-18] roll,
//   [19] enemy_color, [20-28] reserved, [29-30] crc16, [31] tail=0x0d
struct SerialReceiveData {
    // IMU 姿态数据 (弧度)
    float yaw;            // 偏航角 (rad)
    float pitch;          // 俯仰角 (rad)
    float roll;           // 横滚角 (rad)

    // 射击参数
    float bullet_speed;   // 弹速 (m/s)

    // 模式控制
    uint8_t aim_mode;     // 自瞄模式 (原始字节: 0=关闭, 1=自瞄, 2=小符, 3=大符)
    bool aiming_lock;     // 预瞄锁定 (右键按下=true, 释放=false)

    // 协议字段
    uint8_t enemy_color;  // 敌方颜色 (0=未知, 1=红, 2=蓝)

    // 本地开关（不走串口协议）:
    // 当前版本固定为 true（不使用 allow_fire 软门控）
    bool allow_fire;

    // 时间戳 (上位机接收时刻，微秒)
    int64_t recv_time_us = 0;

    SerialReceiveData()
        : yaw(0.0f), pitch(0.0f), roll(0.0f)
        , bullet_speed(15.0f)
        , aim_mode(0), aiming_lock(false)
        , enemy_color(0), allow_fire(true)
        , recv_time_us(0) {}
};

/**
 * @brief 串口发送线程主函数（内部使用）
 * @param transceiver 共享的TransceiverManager实例
 */
void serial_sender_run(std::shared_ptr<TransceiverManager<32>> transceiver);

/**
 * @brief 串口接收线程主函数（内部使用）
 * @param transceiver 共享的TransceiverManager实例
 */
void serial_receiver_run(std::shared_ptr<TransceiverManager<32>> transceiver);

/**
 * @brief 启动串口通信（从配置文件 hardware.toml 读取设置）
 *
 * 根据 Serial.protocol 配置自动选择 UART 或 USB Bulk 协议
 * 支持断线重连
 */
void start_serial_communication();

/**
 * @brief 串口工具类，用于包转换和底层操作
 */
class SerialUtils {
public:
    using PacketType = FixedPacket<32>;

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
