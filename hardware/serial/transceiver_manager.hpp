//
// Created by 霍睿 on 25-3-2.
//

#ifndef TRANSCEIVER_MANAGER_HPP
#define TRANSCEIVER_MANAGER_HPP

// C++ system headers
#include <array>
#include <memory>
#include <mutex>

// Third-party library headers

// Project headers
#include "crc16.hpp"
#include "fixed_packet.hpp"
#include "plugin/debug/logger.hpp"
#include "protocol/protocol_interface.hpp"

namespace serial {

template<std::size_t Capacity = 16>
class TransceiverManager {
public:
    using SharedPtr = std::shared_ptr<TransceiverManager>;
    using PacketType = FixedPacket<Capacity>;

    TransceiverManager() = delete;

    /**
     * @brief 构造函数，创建数据包收发工具
     * @param transporter transport interface
     * @param ignore_crc 是否跳过 CRC 校验 (调试用)
     * @throws std::invalid_argument if transporter is nullptr
     */
    explicit TransceiverManager(std::shared_ptr<ProtocolInterface> transporter, bool ignore_crc = false):
        _transporter(std::move(transporter)),
        _ignore_crc(ignore_crc),
        _recv_buf_len(0) {
        if (!_transporter) {
            throw std::invalid_argument("transporter is nullptr");
        }

        // 初始化缓冲区
        _tmp_buffer.fill(0);
        _recv_buffer.fill(0);

        if (_ignore_crc) {
            debug::print(debug::PrintMode::WARNING, "TransceiverManager",
                "CRC verification DISABLED (debug mode)");
        }
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
     * @brief 线程安全的重连方法
     */
    void safe_reconnect() noexcept;

private:
    std::shared_ptr<ProtocolInterface> _transporter;

    // CRC 校验开关
    bool _ignore_crc;

    // 数据缓冲区（只被接收线程使用，无需保护）
    std::array<uint8_t, Capacity> _tmp_buffer;
    std::array<uint8_t, Capacity * 2> _recv_buffer;
    int _recv_buf_len;

    // 保护重连逻辑的互斥锁
    mutable std::mutex _reconnect_mutex;
};

template<std::size_t Capacity>
bool TransceiverManager<Capacity>::check_packet(
    const uint8_t* buffer,
    int recv_len
) const noexcept {
    // 检查长度
    if (recv_len != static_cast<int>(Capacity)) {
        return false;
    }

    // 检查帧头，帧尾 (使用FixedPacket中定义的常量)
    if ((buffer[0] != PacketType::HEAD_BYTE) || (buffer[Capacity - 1] != PacketType::TAIL_BYTE)) {
        return false;
    }

    // CRC16 校验 (可通过 ignore_crc 跳过)
    if constexpr (Capacity >= 4) {
        if (!_ignore_crc) {
            // CRC 验证: buffer[1] 到 buffer[Capacity-2] (含 CRC)
            // 即 len = Capacity - 2 (不含 head 和 tail)
            if (!crc16_verify(buffer + 1, Capacity - 2)) {
                return false;
            }
        }
    }

    return true;
}

template<std::size_t Capacity>
void TransceiverManager<Capacity>::safe_reconnect() noexcept {
    std::lock_guard<std::mutex> lock(_reconnect_mutex);
    try {
        if (_transporter->is_open()) {
            _transporter->close();
        }
        _transporter->open();
    } catch (const std::exception& e) {
        debug::print(
            debug::PrintMode::ERROR,
            "TransceiverManager",
            "Reconnect failed: {}",
            e.what()
        );
    }
}

template<std::size_t Capacity>
bool TransceiverManager<Capacity>::send_packet(const PacketType& packet) {
    try {
        const auto bytes_written =
            _transporter->write(reinterpret_cast<const std::byte*>(packet.buffer()), Capacity);
        if (bytes_written == static_cast<int>(Capacity)) {
            return true;
        } else {
            // 线程安全的重连
            safe_reconnect();
            return false;
        }
    } catch (const std::exception& e) {
        // 处理可能的异常
        debug::print(
            debug::PrintMode::ERROR,
            "TransceiverManager",
            "Error sending packet: {}",
            e.what()
        );
        return false;
    }
}

template<std::size_t Capacity>
bool TransceiverManager<Capacity>::recv_packet(PacketType& packet) {
    try {
        // read() 操作无需加锁，POSIX 保证并发读写安全
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
                for (int i = 0; (i + static_cast<int> (Capacity)) <= _recv_buf_len; i++) {
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
        } else if (recv_len < 0) {
            // 真正的读取错误，尝试重连
            safe_reconnect();
            return false;
        } else {
            // recv_len == 0: 超时无数据，正常情况
            return false;
        }
    } catch (const std::exception& e) {
        debug::print(
            debug::PrintMode::ERROR,
            "TransceiverManager",
            "Error receiving packet: {}",
            e.what()
        );
        return false;
    }
}

// 常用的固定大小包工具类型别名
using FixedPacketTool16 = TransceiverManager<16>;
using FixedPacketTool32 = TransceiverManager<32>;

} // namespace serial
#endif //TRANSCEIVER_MANAGER_HPP