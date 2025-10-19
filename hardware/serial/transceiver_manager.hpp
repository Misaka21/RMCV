//
// Created by 霍睿 on 25-3-2.
//

#ifndef TRANSCEIVER_MANAGER_HPP
#define TRANSCEIVER_MANAGER_HPP

// C++ system headers
#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <stdexcept>
#include <thread>

// Third-party library headers

// Project headers
#include "fixed_packet.hpp"
#include "protocol/protocol_interface.hpp"
#include "plugin/debug/logger.hpp"

namespace serial {

template<std::size_t Capacity = 16>
class TransceiverManager {
public:
    using SharedPtr = std::shared_ptr<TransceiverManager>;
    using PacketType = FixedPacket<Capacity>;
    enum class SendMode {
        FIFO,           // 先进先出，保留所有包
        LATEST_ONLY,    // 只保留最新的包
        LIMITED_FIFO    // 限制队列大小的FIFO
    };

    TransceiverManager() = delete;

    /**
     * @brief 构造函数，创建固定长度数据包工具
     * @param transporter transport interface
     * @param mode 发送模式，默认为FIFO
     * @param max_queue_size 当mode为LIMITED_FIFO时，队列的最大大小
     * @throws std::invalid_argument if transporter is nullptr
     */
    explicit TransceiverManager(
        std::shared_ptr<ProtocolInterface> transporter,
        SendMode mode = SendMode::FIFO,
        std::size_t max_queue_size = 100)
        : _transporter(std::move(transporter)),
          _recv_buf_len(0),
          _send_mode(mode),
          _max_queue_size(max_queue_size) {
        if (!_transporter) {
            throw std::invalid_argument("transporter is nullptr");
        }

        // 初始化缓冲区
        _tmp_buffer.fill(0);
        _recv_buffer.fill(0);
    }

    /**
     * @brief 析构函数，确保线程正确清理
     */
    ~TransceiverManager() {
        enable_realtime_send(false);
        enable_realtime_read(false);
    }

    /**
     * @brief 检查接口是否打开
     *
     * @return true 已打开，false 未打开
     */
    [[nodiscard]] bool is_open() const noexcept {
        return _transporter->is_open();
    }

    /**
     * @brief 启用/禁用实时发送模式
     *
     * @param enable true启用实时发送，false禁用
     */
    void enable_realtime_send(bool enable);

    /**
     * @brief 发送数据包
     *
     * @param packet 待发送的数据包
     * @return true 发送成功，false 失败
     */
    [[nodiscard]] bool send_packet(const PacketType& packet);

    /**
     * @brief 接收数据包
     *
     * @param packet 输出参数，存储接收到的数据包
     * @return true 接收成功，false 失败
     */
    [[nodiscard]] bool recv_packet(PacketType& packet);

    /**
     * @brief Set the send mode
     *
     * @param mode 发送模式
     * @param max_queue_size 当mode为LIMITED_FIFO时，队列的最大大小
     */
    void set_send_mode(SendMode mode, std::size_t max_queue_size = 100) {
        std::lock_guard<std::mutex> lock(_realtime_send_mut);
        _send_mode = mode;
        _max_queue_size = max_queue_size;

        // 如果切换到只保留最新的模式，清空当前队列
        if (mode == SendMode::LATEST_ONLY && !_realtime_packets.empty()) {
            // 保留最后一个包
            PacketType last_packet = std::move(_realtime_packets.back());
            while (!_realtime_packets.empty()) {
                _realtime_packets.pop();
            }
            _realtime_packets.push(std::move(last_packet));
        }

        // 如果切换到有限大小模式，确保队列不超过限制
        if (mode == SendMode::LIMITED_FIFO && _realtime_packets.size() > _max_queue_size) {
            // 删除旧的包，直到队列大小符合要求
            while (_realtime_packets.size() > _max_queue_size) {
                _realtime_packets.pop();
            }
        }
    }

    /**
     * @brief 启用/禁用实时接收模式
     *
     * @param enable true启用实时接收，false禁用
     */
    void enable_realtime_read(bool enable);

    /**
     * @brief 获取最新接收到的数据包
     *
     * @return std::optional<PacketType> 如果有新数据包则返回，否则返回空
     */
    [[nodiscard]] std::optional<PacketType> get_latest_packet();


private:
    /**
     * @brief 验证接收到的数据包有效性
     *
     * @param buffer 数据缓冲区指针
     * @param recv_len 接收数据长度
     * @return true 数据包有效，false 无效
     */
    [[nodiscard]] bool check_packet(const uint8_t* buffer, int recv_len) const noexcept;

    /**
     * @brief 简单数据包发送实现
     *
     * @param packet 待发送的数据包
     * @return true 发送成功，false 失败
     */
    //[[nodiscard]] 
    bool simple_send_packet(const PacketType& packet);

private:
    std::shared_ptr<ProtocolInterface> _transporter;

    // 数据缓冲区
    std::array<uint8_t, Capacity> _tmp_buffer;
    std::array<uint8_t, Capacity * 2> _recv_buffer;
    int _recv_buf_len;

    // 实时发送相关
    std::atomic<bool> _use_realtime_send{false};
    std::mutex _realtime_send_mut;
    std::unique_ptr<std::thread> _realtime_send_thread;
    std::queue<PacketType> _realtime_packets;

    // 实时接收相关
    std::atomic<bool> _use_realtime_read{false};
    std::mutex _realtime_read_mut;
    std::unique_ptr<std::thread> _realtime_read_thread;
    std::optional<PacketType> _latest_packet;

    // 发送模式配置
    SendMode _send_mode{SendMode::FIFO};
    std::size_t _max_queue_size{100};
};

template<std::size_t Capacity>
bool TransceiverManager<Capacity>::check_packet(const uint8_t* buffer, int recv_len) const noexcept {
    // 检查长度
    if (recv_len != static_cast<int>(Capacity)) {
        return false;
    }

    // 检查帧头，帧尾 (使用FixedPacket中定义的常量)
    if ((buffer[0] != PacketType::HEAD_BYTE) || (buffer[Capacity - 1] != PacketType::TAIL_BYTE)) {
        return false;
    }

    // TODO(gezp): 检查check_byte(buffer[capacity-2]),可采用异或校验(BCC)
    return true;
}

template<std::size_t Capacity>
bool TransceiverManager<Capacity>::simple_send_packet(const PacketType& packet) {
    try {
        const auto bytes_written =
            _transporter->write(reinterpret_cast<const std::byte*>(packet.buffer()), Capacity);
        if (bytes_written == static_cast<int>(Capacity)) {
            return true;
        } else {
            // 尝试重新连接
            _transporter->close();
            _transporter->open();
            return false;
        }
    } catch (const std::exception& e) {
        // 处理可能的异常
        debug::print(debug::PrintMode::ERROR, "TransceiverManager", "Error sending packet: {}", e.what());
        return false;
    }
}

template<std::size_t Capacity>
void TransceiverManager<Capacity>::enable_realtime_send(bool enable) {
    // 如果状态未改变，直接返回
    if (enable == _use_realtime_send) {
        return;
    }

    if (enable) {
        _use_realtime_send = true;
        _realtime_send_thread = std::make_unique<std::thread>([this]() {
            PacketType packet;
            using namespace std::chrono_literals;

            while (_use_realtime_send) {
                bool empty = true;
                {
                    std::lock_guard<std::mutex> lock(_realtime_send_mut);
                    empty = _realtime_packets.empty();
                    if (!empty) {
                        packet = std::move(_realtime_packets.front());
                        _realtime_packets.pop();
                    }
                }

                if (!empty) {
                    simple_send_packet(packet);
                } else {
                    std::this_thread::sleep_for(1ms);
                }
            }
        });
    } else {
        _use_realtime_send = false;
        if (_realtime_send_thread && _realtime_send_thread->joinable()) {
            _realtime_send_thread->join();
            _realtime_send_thread.reset();
        }
    }
}

template<std::size_t Capacity>
bool TransceiverManager<Capacity>::send_packet(const PacketType& packet) {
    if (_use_realtime_send) {
        try {
            std::lock_guard<std::mutex> lock(_realtime_send_mut);

            switch (_send_mode) {
                case SendMode::LATEST_ONLY:
                    // 仅保留最新的包
                        while (!_realtime_packets.empty()) {
                            _realtime_packets.pop();
                        }
                _realtime_packets.push(packet);
                break;

                case SendMode::LIMITED_FIFO:
                    // 限制队列大小的FIFO
                        if (_realtime_packets.size() >= _max_queue_size) {
                            // 队列已满，移除最早的包
                            _realtime_packets.pop();
                        }
                _realtime_packets.push(packet);
                break;

                case SendMode::FIFO:
                    default:
                        // 默认行为：先进先出
                        _realtime_packets.push(packet);
                break;
            }
            return true;
        } catch (const std::exception& e) {
            debug::print(debug::PrintMode::ERROR, "TransceiverManager", "Error queuing packet: {}", e.what());
            return false;
        }
    } else {
        return simple_send_packet(packet);
    }
}

template<std::size_t Capacity>
bool TransceiverManager<Capacity>::recv_packet(PacketType& packet) {
    try {
        int recv_len =
            _transporter->read(reinterpret_cast<std::byte*>(_tmp_buffer.data()), Capacity);
        if (recv_len > 0) {
            // 检查是否是完整数据包
            if (check_packet(_tmp_buffer.data(), recv_len)) {
                packet.copy_from(_tmp_buffer.data());
                return true;
            } else {
                // 如果是断帧，拼接缓存，并遍历校验，获得合法数据
                if (_recv_buf_len + recv_len > static_cast<int>(Capacity * 2)) {
                    _recv_buf_len = 0; // 缓冲区溢出时重置
                }

                // 拼接缓存
                std::memcpy(_recv_buffer.data() + _recv_buf_len, _tmp_buffer.data(), recv_len);
                _recv_buf_len += recv_len;

                // 遍历校验
                for (int i = 0; (i + Capacity) <= _recv_buf_len; i++) {
                    if (check_packet(_recv_buffer.data() + i, Capacity)) {
                        packet.copy_from(_recv_buffer.data() + i);

                        // 读取一帧后，更新接收缓存
                        int k = 0;
                        for (int j = i + Capacity; j < _recv_buf_len; j++, k++) {
                            _recv_buffer[k] = _recv_buffer[j];
                        }
                        _recv_buf_len = k;
                        return true;
                    }
                }

                // 表明断帧，或错误帧
                return false;
            }
        } else {
            // 尝试重新连接
            _transporter->close();
            _transporter->open();
            return false;
        }
    } catch (const std::exception& e) {
        debug::print(debug::PrintMode::ERROR, "TransceiverManager", "Error receiving packet: {}", e.what());
        return false;
    }
}

template<std::size_t Capacity>
void TransceiverManager<Capacity>::enable_realtime_read(bool enable) {
    // 如果状态未改变，直接返回
    if (enable == _use_realtime_read) {
        return;
    }

    if (enable) {
        _use_realtime_read = true;
        _realtime_read_thread = std::make_unique<std::thread>([this]() {
            PacketType packet;
            using namespace std::chrono_literals;

            while (_use_realtime_read) {
                if (recv_packet(packet)) {
                    // 接收成功，更新最新的数据包
                    std::lock_guard<std::mutex> lock(_realtime_read_mut);
                    _latest_packet = packet;
                } else {
                    // 如果没有接收到数据，短暂休眠以避免CPU占用过高
                    std::this_thread::sleep_for(1ms);
                }
            }
        });
    } else {
        _use_realtime_read = false;
        if (_realtime_read_thread && _realtime_read_thread->joinable()) {
            _realtime_read_thread->join();
            _realtime_read_thread.reset();
        }
    }
}

template<std::size_t Capacity>
auto TransceiverManager<Capacity>::get_latest_packet()->std::optional<PacketType> {
    std::lock_guard<std::mutex> lock(_realtime_read_mut);
    std::optional<PacketType> result = std::nullopt;

    if (_latest_packet.has_value()) {
        result = _latest_packet;
        // 可选：读取后清除，取决于您的需求
        // _latest_packet = std::nullopt;
    }

    return result;
}

// 常用的固定大小包工具类型别名
using FixedPacketTool16 = TransceiverManager<16>;
using FixedPacketTool32 = TransceiverManager<32>;
using FixedPacketTool64 = TransceiverManager<64>;

} // namespace serial
#endif //TRANSCEIVER_MANAGER_HPP
