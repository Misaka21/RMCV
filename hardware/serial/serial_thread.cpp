//
// Created by 霍睿 on 25-3-2.
//
#include <chrono>
#include <thread>

#include "plugin/debug/logger.hpp"
#include "plugin/param/static_config.hpp"
#include "plugin/stats/fps_stats.hpp"
#include "protocol/uart_protocol.hpp"
#include "protocol/usb_bulk_protocol.hpp"
#include "serial_thread.hpp"

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
        auto app_running = umt::BasicObjManager<bool>::find_or_create("app_running", true);

        debug::print(debug::PrintMode::INFO, "SerialSender", "Sender thread started");

        // FPS 统计: FPS=循环次数, sent=实际发送成功次数
        stats::FpsStats fps_stats("SerialSender", "sent");

        while (app_running->get()) {
            try {
                // 检查发送是否启用
                if (!send_enabled->get()) {
                    std::this_thread::sleep_for(10ms);
                    continue;
                }

                // 从ObjManager获取视觉数据
                VisionData_t vision_data = vision_transmit->get();

                // 转换为数据包并发送
                bool sent = false;
                FixedPacket<16> packet;
                if (SerialUtils::vision_data_to_packet(vision_data, packet)) {
                    if (transceiver->send_packet(packet)) {
                        sent = true;
                    }
                }

                // 更新统计: 每次循环计数，发送成功时 secondary_hit=true
                fps_stats.update(0, sent);

                // 短暂休眠避免过度占用CPU
                std::this_thread::sleep_for(1ms);

            } catch (const std::exception& e) {
                debug::print(debug::PrintMode::ERROR, "SerialSender", "Exception: {}", e.what());
                std::this_thread::sleep_for(100ms);
            }
        }

        debug::print(debug::PrintMode::INFO, "SerialSender", "Sender thread stopped");

    } catch (const std::exception& e) {
        debug::print(debug::PrintMode::ERROR, "SerialSender", "Init failed: {}", e.what());
    }
}

void serial_receiver_run(std::shared_ptr<TransceiverManager<16>> transceiver) {
    try {
        // 创建接收数据队列并通过BasicObjManager共享
        auto receive_queue = umt::BasicObjManager<ReceiveQueue>::find_or_create("receive_queue");
        auto recv_enabled = umt::BasicObjManager<bool>::find_or_create("serial_recv_enabled", true);
        auto app_running = umt::BasicObjManager<bool>::find_or_create("app_running", true);

        debug::print(debug::PrintMode::INFO, "SerialReceiver", "Receiver thread started");

        // FPS 统计: FPS=循环次数, received=实际收到数据次数
        stats::FpsStats fps_stats("SerialReceiver", "received");

        while (app_running->get()) {
            try {
                // 检查接收是否启用
                if (!recv_enabled->get()) {
                    std::this_thread::sleep_for(10ms);
                    continue;
                }

                // 接收数据包
                FixedPacket<16> packet;
                bool received = transceiver->recv_packet(packet);

                if (received) {
                    // 立即记录接收时间戳 (关键: 减少延迟抖动)
                    auto recv_time = std::chrono::steady_clock::now();
                    int64_t recv_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
                        recv_time.time_since_epoch()
                    ).count();

                    // 转换为数据结构体
                    SerialReceiveData receive_data;
                    if (SerialUtils::packet_to_receive_data(packet, receive_data)) {
                        // 设置时间戳
                        receive_data.recv_time_us = recv_time_us;

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

                // 更新统计: 每次循环计数，收到数据时 secondary_hit=true
                fps_stats.update(0, received);

            } catch (const std::exception& e) {
                debug::print(debug::PrintMode::ERROR, "SerialReceiver", "Exception: {}", e.what());
                std::this_thread::sleep_for(10ms);
            }
        }

        debug::print(debug::PrintMode::INFO, "SerialReceiver", "Receiver thread stopped");

    } catch (const std::exception& e) {
        debug::print(debug::PrintMode::ERROR, "SerialReceiver", "Init failed: {}", e.what());
    }
}

/**
 * @brief 串口管理器 - 负责创建和管理串口实例
 */
class SerialManager {
public:
    SerialManager() = delete;

    // 重试配置
    static constexpr int MAX_RETRY_COUNT = 5;
    static constexpr int RETRY_INTERVAL_MS = 2000;

    /**
     * @brief 根据配置文件创建协议实例
     * @param config 解析后的配置
     * @return 协议实例，失败返回 nullptr
     */
    static std::shared_ptr<ProtocolInterface> create_protocol_from_config(const toml::table& config) {
        std::string protocol_type = static_param::get_param<std::string>(config, "Serial", "protocol");

        if (protocol_type == "uart") {
            return create_uart_protocol(config);
        } else if (protocol_type == "usb_bulk") {
            return create_usb_bulk_protocol(config);
        } else {
            debug::print(debug::PrintMode::ERROR, "SerialManager",
                "Unknown protocol type: {}", protocol_type);
            return nullptr;
        }
    }

    /**
     * @brief 创建 UART 协议
     */
    static std::shared_ptr<ProtocolInterface> create_uart_protocol(const toml::table& config) {
        std::string port_name = static_param::get_param<std::string>(config, "Serial.uart", "port_name");
        int64_t baudrate = static_param::get_param<int64_t>(config, "Serial.uart", "baudrate");

        debug::print(debug::PrintMode::INFO, "SerialManager",
            "Creating UART: {} @ {}", port_name, baudrate);

        return std::make_shared<UartProtocol>(port_name, static_cast<int>(baudrate));
    }

    /**
     * @brief 创建 USB Bulk 协议
     */
    static std::shared_ptr<ProtocolInterface> create_usb_bulk_protocol(const toml::table& config) {
        std::string vendor_id = static_param::get_param<std::string>(config, "Serial.usb_bulk", "vendor_id");
        std::string product_id = static_param::get_param<std::string>(config, "Serial.usb_bulk", "product_id");
        std::string serial_number = static_param::get_param<std::string>(config, "Serial.usb_bulk", "serial_number");
        int64_t interface_number = static_param::get_param<int64_t>(config, "Serial.usb_bulk", "interface_number");
        std::string bulk_in = static_param::get_param<std::string>(config, "Serial.usb_bulk", "bulk_in_endpoint");
        std::string bulk_out = static_param::get_param<std::string>(config, "Serial.usb_bulk", "bulk_out_endpoint");
        int64_t timeout_ms = static_param::get_param<int64_t>(config, "Serial.usb_bulk", "timeout_ms");

        debug::print(debug::PrintMode::INFO, "SerialManager",
            "Creating USB Bulk: VID={} PID={}", vendor_id, product_id.empty() ? "(any)" : product_id);

        return UsbBulkProtocol::create_from_config(
            vendor_id,
            product_id,
            serial_number,
            static_cast<int>(interface_number),
            bulk_in,
            bulk_out,
            static_cast<int>(timeout_ms)
        );
    }

    /**
     * @brief 启动串口收发线程（从配置文件读取设置）
     *
     * 如果串口打开失败，会重试
     * 重试全部失败后程序退出
     */
    static void start_serial_threads() {
        // 加载配置
        auto config = static_param::parse_file("hardware.toml");

        std::shared_ptr<ProtocolInterface> protocol = nullptr;
        int retry_count = 0;
        int64_t reconnect_interval_ms = RETRY_INTERVAL_MS;
        int64_t max_reconnect = MAX_RETRY_COUNT;

        // USB Bulk 使用配置的重连参数
        std::string protocol_type = static_param::get_param<std::string>(config, "Serial", "protocol");
        if (protocol_type == "usb_bulk") {
            reconnect_interval_ms = static_param::get_param<int64_t>(config, "Serial.usb_bulk", "reconnect_interval_ms");
            max_reconnect = static_param::get_param<int64_t>(config, "Serial.usb_bulk", "max_reconnect_attempts");
        }

        // 重试打开串口
        while (max_reconnect < 0 || retry_count < max_reconnect) {
            try {
                protocol = create_protocol_from_config(config);

                if (protocol && protocol->open()) {
                    debug::print(debug::PrintMode::INFO, "SerialManager", "Protocol opened successfully");
                    break;
                }

                if (protocol) {
                    debug::print(debug::PrintMode::WARNING, "SerialManager",
                        "Open failed ({}/{}): {}", retry_count + 1,
                        max_reconnect < 0 ? "inf" : std::to_string(max_reconnect),
                        protocol->error_message());
                }

            } catch (const std::exception& e) {
                debug::print(debug::PrintMode::WARNING, "SerialManager",
                    "Exception ({}/{}): {}", retry_count + 1,
                    max_reconnect < 0 ? "inf" : std::to_string(max_reconnect), e.what());
            }

            retry_count++;
            debug::print(debug::PrintMode::INFO, "SerialManager",
                "Retry in {} ms...", reconnect_interval_ms);
            std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_interval_ms));
        }

        // 重试全部失败，退出程序
        if (!protocol || !protocol->is_open()) {
            debug::print(debug::PrintMode::FATAL, "SerialManager",
                "Protocol open failed after {} retries, exiting", retry_count);
            std::exit(1);
        }

        try {
            // 创建TransceiverManager（共享）
            auto transceiver = std::make_shared<TransceiverManager<16>>(protocol);

            // 启动发送线程
            std::thread([transceiver]() { serial_sender_run(transceiver); }).detach();

            // 启动接收线程
            std::thread([transceiver]() { serial_receiver_run(transceiver); }).detach();

            debug::print(debug::PrintMode::INFO, "SerialManager", "TX/RX threads started");

        } catch (const std::exception& e) {
            debug::print(debug::PrintMode::FATAL, "SerialManager", "Start failed: {}", e.what());
            std::exit(1);
        }
    }

    /**
     * @brief 启动串口收发线程（指定端口和波特率，向后兼容）
     * @param port_path 串口设备路径
     * @param baud_rate 波特率
     */
    static void start_serial_threads(const std::string& port_path, int baud_rate) {
        debug::print(debug::PrintMode::INFO, "SerialManager", "Starting UART: {} @ {}", port_path, baud_rate);

        std::shared_ptr<UartProtocol> uart = nullptr;
        int retry_count = 0;

        // 重试打开串口
        while (retry_count < MAX_RETRY_COUNT) {
            try {
                uart = std::make_shared<UartProtocol>(port_path, baud_rate);

                if (uart->open()) {
                    debug::print(debug::PrintMode::INFO, "SerialManager", "Port {} opened", port_path);
                    break;
                }

                debug::print(debug::PrintMode::WARNING, "SerialManager",
                    "Open failed ({}/{}): {}", retry_count + 1, MAX_RETRY_COUNT, uart->error_message());

            } catch (const std::exception& e) {
                debug::print(debug::PrintMode::WARNING, "SerialManager",
                    "Exception ({}/{}): {}", retry_count + 1, MAX_RETRY_COUNT, e.what());
            }

            retry_count++;
            if (retry_count < MAX_RETRY_COUNT) {
                debug::print(debug::PrintMode::INFO, "SerialManager",
                    "Retry in {} seconds...", RETRY_INTERVAL_MS / 1000);
                std::this_thread::sleep_for(std::chrono::milliseconds(RETRY_INTERVAL_MS));
            }
        }

        // 重试全部失败，退出程序
        if (!uart || !uart->is_open()) {
            debug::print(debug::PrintMode::FATAL, "SerialManager",
                "Port {} open failed after {} retries, exiting", port_path, MAX_RETRY_COUNT);
            std::exit(1);
        }

        try {
            // 创建TransceiverManager（共享）
            auto transceiver = std::make_shared<TransceiverManager<16>>(uart);

            // 启动发送线程
            std::thread([transceiver]() { serial_sender_run(transceiver); }).detach();

            // 启动接收线程
            std::thread([transceiver]() { serial_receiver_run(transceiver); }).detach();

            debug::print(debug::PrintMode::INFO, "SerialManager", "TX/RX threads started");

        } catch (const std::exception& e) {
            debug::print(debug::PrintMode::FATAL, "SerialManager", "Start failed: {}", e.what());
            std::exit(1);
        }
    }
};

/**
 * @brief 启动串口通信（从配置文件读取设置）
 */
void start_serial_communication() {
    SerialManager::start_serial_threads();
}

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
        debug::print(debug::PrintMode::ERROR, "SerialUtils", "vision_data_to_packet: {}", e.what());
        return false;
    }
}

bool SerialUtils::packet_to_receive_data(const PacketType& packet, SerialReceiveData& data) {
    try {
        // 从数据包提取数据 (格式需与电控约定)
        // 目前简单解析: [1]yaw [5]pitch [9]roll
        float yaw, pitch, roll;
        if (packet.unload_data(yaw, 1)) {
            data.yaw = yaw;
        }
        if (packet.unload_data(pitch, 5)) {
            data.pitch = pitch;
        }
        if (packet.unload_data(roll, 9)) {
            data.roll = roll;
        }
        // TODO: 其他字段需要和电控约定后添加

        return true;
    } catch (const std::exception& e) {
        debug::print(debug::PrintMode::ERROR, "SerialUtils", "packet_to_receive_data: {}", e.what());
        return false;
    }
}

} // namespace serial
