//
// Created by 霍睿 on 25-3-2.
//
#include <chrono>
#include <thread>

#include "crc16.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/param/static_config.hpp"
#include "plugin/stats/fps_stats.hpp"
#include "protocol/uart_protocol.hpp"
#ifdef HAVE_LIBUSB_1_0
#include "protocol/usb_bulk_protocol.hpp"
#endif
#include "serial_thread.hpp"

// FireCommand -> VisionData_t bridge
#include "aimer/common/fire_control_types.hpp"

// UMT相关头文件
#include "umt/umt.hpp"

namespace serial {

namespace umt = ::umt;
using namespace std::chrono_literals;

void serial_sender_run(std::shared_ptr<TransceiverManager<32>> transceiver) {
    try {
        // 输出到下位机的数据源: fire_command (autoaim/autobuff 写入)
        auto fire_cmd = umt::BasicObjManager<::fire_control::FireCommand>::find_or_create("fire_command");
        // 兼容/调试: 暴露一个“最终将要发送”的 VisionData_t 供其他模块查看
        auto vision_transmit = umt::BasicObjManager<VisionData_t>::find_or_create("vision_transmit");
        auto send_enabled = umt::BasicObjManager<bool>::find_or_create("serial_send_enabled", true);
        auto app_running = umt::BasicObjManager<bool>::find_or_create("app_running", true);
        auto debug_print = umt::BasicObjManager<bool>::find_or_create("serial_debug_print", false);

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

                // fire_command -> VisionData_t (保持 32B 协议不变)
                const auto cmd = fire_cmd->get();
                VisionData_t vision_data;
                vision_data.control = cmd.control_enabled ? 1 : 0;
                vision_data.shoot = (cmd.control_enabled && cmd.allow_fire && cmd.fire_now) ? 1 : 0;
                vision_data.yaw = cmd.yaw;
                vision_data.pitch = cmd.pitch;

                // 写回调试对象
                vision_transmit->get() = vision_data;

                // 转换为数据包并发送
                bool sent = false;
                FixedPacket<32> packet;
                if (SerialUtils::vision_data_to_packet(vision_data, packet)) {
                    // 调试打印
                    if (debug_print->get()) {
                        std::string hex;
                        for (size_t i = 0; i < 32; ++i) {
                            hex += fmt::format("{:02X} ", packet.buffer()[i]);
                        }
                        debug::print(debug::PrintMode::DEBUG, "SerialTX", "{}", hex);
                    }

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

void serial_receiver_run(std::shared_ptr<TransceiverManager<32>> transceiver) {
    try {
        // 使用 Message 系统发布接收数据（线程安全）
        umt::Publisher<SerialReceiveData> publisher("serial_receive");
        auto recv_enabled = umt::BasicObjManager<bool>::find_or_create("serial_recv_enabled", true);
        auto app_running = umt::BasicObjManager<bool>::find_or_create("app_running", true);
        auto debug_print = umt::BasicObjManager<bool>::find_or_create("serial_debug_print", false);

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
                FixedPacket<32> packet;
                bool received = transceiver->recv_packet(packet);

                if (received) {
                    // 调试打印
                    if (debug_print->get()) {
                        std::string hex;
                        for (size_t i = 0; i < 32; ++i) {
                            hex += fmt::format("{:02X} ", packet.buffer()[i]);
                        }
                        debug::print(debug::PrintMode::DEBUG, "SerialRX", "{}", hex);
                    }

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

                        // 通过 Message 发布（线程安全，Subscriber 自动管理缓冲区）
                        publisher.push(receive_data);
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
#ifdef HAVE_LIBUSB_1_0
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
#else
        (void)config;
        debug::print(debug::PrintMode::ERROR, "SerialManager",
            "USB bulk protocol requested but this build has no libusb-1.0 support (HAVE_LIBUSB_1_0 is not defined)");
        return nullptr;
#endif
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

        // 读取调试配置
        bool ignore_crc = static_param::get_param<bool>(config, "Serial", "ignore_crc");
        bool data_print_debug = static_param::get_param<bool>(config, "Serial", "data_print_debug");

        // 设置调试打印标志 (供收发线程使用)
        auto debug_print = umt::BasicObjManager<bool>::find_or_create("serial_debug_print", data_print_debug);
        debug_print->get() = data_print_debug;

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
            auto transceiver = std::make_shared<TransceiverManager<32>>(protocol, ignore_crc);

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

// SerialUtils实现
// VisionToBoard 协议布局 (视觉 → 电控, 32字节):
//   [0] head=0xff, [1] control, [2] shoot,
//   [3-6] yaw, [7-10] pitch, [11-28] reserved,
//   [29-30] crc16, [31] tail=0x0d
// CRC 计算范围: buffer[1..28] (28字节)
bool SerialUtils::vision_data_to_packet(const VisionData_t& cmd, PacketType& packet) {
    try {
        // 清空数据包
        packet.clear();

        // [1] control
        packet.load_data(cmd.control, 1);

        // [2] shoot
        packet.load_data(cmd.shoot, 2);

        // [3-6] yaw (弧度)
        packet.load_data(cmd.yaw, 3);

        // [7-10] pitch (弧度)
        packet.load_data(cmd.pitch, 7);

        // [11-28] reserved (默认0)

        // 计算并填充 CRC16
        // CRC 范围: buffer[1..28]，存入 buffer[29..30]
        uint8_t* buf = const_cast<uint8_t*>(packet.buffer());
        crc16_append(buf + 1, 30);  // 计算 [1..28]，写入 [29..30]

        return true;
    } catch (const std::exception& e) {
        debug::print(debug::PrintMode::ERROR, "SerialUtils", "vision_data_to_packet: {}", e.what());
        return false;
    }
}

// BoardToVision 协议布局 (电控 → 视觉, 32字节):
//   [0] head=0xff, [1] mode, [2] aiming_lock,
//   [3-6] bullet_speed, [7-10] yaw, [11-14] pitch, [15-18] roll,
//   [19-28] reserved, [29-30] crc16, [31] tail=0x0d
// CRC 计算范围: buffer[1..28] (28字节)
bool SerialUtils::packet_to_receive_data(const PacketType& packet, SerialReceiveData& data) {
    try {
        const uint8_t* buf = packet.buffer();

        // 验证 CRC16
        // CRC 范围: buffer[1..30] (含 CRC 本身)
        if (!crc16_verify(buf + 1, 30)) {
            debug::print(debug::PrintMode::WARNING, "SerialUtils", "CRC16 verification failed");
            return false;
        }

        // [1] mode
        uint8_t mode = 0;
        if (packet.unload_data(mode, 1)) {
            data.aim_mode = mode;
        }

        // [2] aiming_lock
        uint8_t aiming_lock = 0;
        if (packet.unload_data(aiming_lock, 2)) {
            data.aiming_lock = (aiming_lock != 0);
        }

        // [3-6] bullet_speed (m/s)
        float bullet_speed = 15.0f;
        if (packet.unload_data(bullet_speed, 3)) {
            data.bullet_speed = bullet_speed;
        }

        // [7-10] yaw (弧度)
        float yaw = 0.0f;
        if (packet.unload_data(yaw, 7)) {
            data.yaw = yaw;
        }

        // [11-14] pitch (弧度)
        float pitch = 0.0f;
        if (packet.unload_data(pitch, 11)) {
            data.pitch = pitch;
        }

        // [15-18] roll (弧度)
        float roll = 0.0f;
        if (packet.unload_data(roll, 15)) {
            data.roll = roll;
        }

        return true;
    } catch (const std::exception& e) {
        debug::print(debug::PrintMode::ERROR, "SerialUtils", "packet_to_receive_data: {}", e.what());
        return false;
    }
}

} // namespace serial
