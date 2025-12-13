//
// Created by 霍睿 on 25-3-2.
//
#include <chrono>
#include <iostream>
#include <thread>
#include <fmt/color.h>
#include <fmt/format.h>

#include "serial_node.hpp"
#include "protocol/uart_protocol.hpp"
#include "plugin/debug/logger.hpp"

// UMT相关头文件
#include "umt/umt.hpp"

namespace serial {

namespace umt = ::umt;
using namespace std::chrono_literals;

void serial_sender_run(std::shared_ptr<TransceiverManager<16>> transceiver) {
    try {
        // 视觉数据状态管理
        auto vision_transmit = umt::BasicObjManager<VisionData_t>::find_or_create("vision_transmit");
        auto send_enabled = umt::BasicObjManager<bool>::find_or_create("serial_send_enabled", true);

        fmt::print(fmt::fg(fmt::color::green), "[INFO] 串口发送线程启动成功\n");

        int fps = 0, fps_count = 0;
        auto t1 = std::chrono::system_clock::now();

        while (true) {
            try {
                // 检查发送是否启用
                if (!send_enabled->get()) {
                    std::this_thread::sleep_for(10ms);
                    continue;
                }

                // 从ObjManager获取视觉数据
                VisionData_t vision_data = vision_transmit->get();

                // 转换为数据包
                FixedPacket<16> packet;
                if (SerialUtils::vision_data_to_packet(vision_data, packet)) {
                    // 发送数据包
                    if (!transceiver->send_packet(packet)) {
                        fmt::print(fmt::fg(fmt::color::yellow), "[WARNING] 发送数据包失败\n");
                    } else {
                        // 更新FPS统计
                        fps_count++;
                        auto t2 = std::chrono::system_clock::now();
                        if (std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() >= 1000) {
                            fps = fps_count;
                            fps_count = 0;
                            t1 = t2;
                            fmt::print(fmt::fg(fmt::color::cyan), "[INFO] 串口发送帧率: {} fps\n", fps);
                        }
                    }
                } else {
                    fmt::print(fmt::fg(fmt::color::yellow), "[WARNING] 视觉数据转换失败\n");
                }

                // 短暂休眠避免过度占用CPU
                std::this_thread::sleep_for(5ms);

            } catch (const std::exception& e) {
                fmt::print(fmt::fg(fmt::color::red), "[ERROR] 发送线程异常: {}\n", e.what());
                std::this_thread::sleep_for(100ms);
            }
        }

    } catch (const std::exception& e) {
        fmt::print(fmt::fg(fmt::color::red), "[ERROR] 串口发送线程初始化失败: {}\n", e.what());
    }
}

void serial_receiver_run(std::shared_ptr<TransceiverManager<16>> transceiver) {
    try {
        // 创建接收数据队列并通过BasicObjManager共享
        auto receive_queue = umt::BasicObjManager<ReceiveQueue>::find_or_create("receive_queue");
        auto recv_enabled = umt::BasicObjManager<bool>::find_or_create("serial_recv_enabled", true);

        fmt::print(fmt::fg(fmt::color::green), "[INFO] 串口接收线程启动成功\n");

        while (true) {
            try {
                // 检查接收是否启用
                if (!recv_enabled->get()) {
                    std::this_thread::sleep_for(10ms);
                    continue;
                }

                // 接收数据包
                FixedPacket<16> packet;
                if (transceiver->recv_packet(packet)) {
                    // 转换为数据结构体
                    SerialReceiveData receive_data;
                    if (SerialUtils::packet_to_receive_data(packet, receive_data)) {
                        // 添加时间戳
                        receive_data.timestamp =
                            std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::steady_clock::now().time_since_epoch()
                            ).count();

                        // 限制队列大小，最多300条
                        if (receive_queue->get().size() >= 300) {
                            receive_queue->get().pop();  // 移除最旧的
                        }

                        // 添加到队列
                        receive_queue->get().push(receive_data);
                    }
                } else {
                    std::this_thread::sleep_for(1ms);
                }

            } catch (const std::exception& e) {
                fmt::print(fmt::fg(fmt::color::red), "[ERROR] 接收线程异常: {}\n", e.what());
                std::this_thread::sleep_for(10ms);
            }
        }

    } catch (const std::exception& e) {
        fmt::print(fmt::fg(fmt::color::red), "[ERROR] 串口接收线程初始化失败: {}\n", e.what());
    }
}

/**
 * @brief 串口管理器 - 负责创建和管理串口实例
 */
class SerialManager {
public:
    SerialManager() = delete;

    /**
     * @brief 启动串口收发线程（共享同一个串口实例）
     * @param port_path 串口设备路径
     * @param baud_rate 波特率
     */
    static void start_serial_threads(const std::string& port_path = "/dev/ttyUSB0", int baud_rate = 115200) {
        fmt::print(fmt::fg(fmt::color::green), "[INFO] 正在启动串口管理器: {} @ {}\n", port_path, baud_rate);

        try {
            // 创建串口协议实例（共享）
            auto uart = std::make_shared<UartProtocol>(port_path, baud_rate);

            // 尝试打开串口
            if (!uart->open()) {
                fmt::print(fmt::fg(fmt::color::red), "[ERROR] 无法打开串口 {}: {}\n",
                          port_path, uart->error_message());
                return;
            }

            // 创建TransceiverManager（共享）
            auto transceiver = std::make_shared<TransceiverManager<16>>(uart);

            // 启动发送线程
            std::thread([transceiver]() { serial_sender_run(transceiver); }).detach();

            // 启动接收线程
            std::thread([transceiver]() { serial_receiver_run(transceiver); }).detach();

            fmt::print(fmt::fg(fmt::color::green), "[INFO] 串口收发线程启动完成\n");

        } catch (const std::exception& e) {
            fmt::print(fmt::fg(fmt::color::red), "[ERROR] 串口管理器启动失败: {}\n", e.what());
        }
    }
};

/**
 * @brief 启动串口通信（同时启动发送和接收线程，共享串口实例）
 * @param port_path 串口设备路径
 * @param baud_rate 波特率
 */
void start_serial_communication(const std::string& port_path, int baud_rate) {
    SerialManager::start_serial_threads(port_path, baud_rate);
}

// SerialUtils实现
bool SerialUtils::vision_data_to_packet(const VisionData_t& cmd, PacketType& packet) {
    try {
        // 清空数据包
        packet.clear();

        // 填充数据（根据实际协议调整格式）
        packet.load_data(static_cast<float>(cmd.cmd_id), 1);
        packet.load_data(cmd.yaw, 5);
        packet.load_data(cmd.pitch, 9);
        packet.load_data(cmd.distance, 13);

        return true;
    } catch (const std::exception& e) {
        debug::print(debug::PrintMode::ERROR, "SerialUtils", "Error in vision_data_to_packet: {}", e.what());
        return false;
    }
}

bool SerialUtils::packet_to_receive_data(const PacketType& packet, SerialReceiveData& data) {
    try {
        // 从数据包提取数据
        float cmd_id_float, yaw, pitch, distance;
        if (packet.unload_data(cmd_id_float, 1)) {
            data.cmd_id = static_cast<uint8_t>(cmd_id_float);
        }
        if (packet.unload_data(yaw, 5)) {
            data.yaw = yaw;
        }
        if (packet.unload_data(pitch, 9)) {
            data.pitch = pitch;
        }
        if (packet.unload_data(distance, 13)) {
            data.distance = distance;
        }

        return true;
    } catch (const std::exception& e) {
        debug::print(debug::PrintMode::ERROR, "SerialUtils", "Error in packet_to_receive_data: {}", e.what());
        return false;
    }
}

} // namespace serial