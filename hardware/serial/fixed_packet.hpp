//
// Created by 霍睿 on 25-3-2.
//

#ifndef FIXED_PACKET_HPP
#define FIXED_PACKET_HPP

#include <cstring>
#include <memory>
#include <array>
#include <optional>
#include <stdexcept>
#include <type_traits>

namespace serial {

// 定长数据包封装
// [head_byte(0xff),...(data_bytes)...,check_byte,tail_byte(0x0d)]
template<std::size_t Capacity = 16>
class FixedPacket {
public:
    using SharedPtr = std::shared_ptr<FixedPacket>;
    
    static_assert(Capacity >= 3, "Packet capacity must be at least 3 bytes");
    
    constexpr static uint8_t HEAD_BYTE = 0xff;
    constexpr static uint8_t TAIL_BYTE = 0x0d;

    FixedPacket() {
        _buffer.fill(0);
        _buffer[0] = HEAD_BYTE;                // 帧头
        _buffer[Capacity - 1] = TAIL_BYTE;     // 帧尾
    }
    
    /**
     * @brief Flush buffer
     * 清除缓存, date_bytes和check_byte都用0填充
     */
    void clear() noexcept {
        std::fill(_buffer.begin() + 1, _buffer.end() - 1, 0);
    }

    /**
     * @brief Set the check byte
     * 设置校验字节
     * @param check_byte
     */
    void set_check_byte(uint8_t check_byte) noexcept {
        _buffer[Capacity - 2] = check_byte;
    }

    /**
     * @brief Copy data to buffer
     * copy数据到缓存buffer
     * @param src 源数据指针
     * @throws std::invalid_argument 如果src为nullptr
     */
    void copy_from(const void* src) {
        if (src == nullptr) {
            throw std::invalid_argument("Source pointer cannot be null");
        }
        std::memcpy(_buffer.data(), src, Capacity);
    }

    /**
     * @brief Get buffer
     * 获取缓存buffer
     * @return const uint8_t*
     */
    [[nodiscard]] const uint8_t* buffer() const noexcept {
        return _buffer.data();
    }

    /**
     * @brief Self-define data loader
     * 自定义装载数据
     * @tparam T Data type
     * @tparam DataLen Data length, defaults to sizeof(T)
     * @param data 需要装载的数据
     * @param index 目标位置
     * @return std::optional<bool> 成功返回true，失败返回std::nullopt
     */
    template<typename T, std::size_t DataLen = sizeof(T)>
    std::optional<bool> load_data(T const& data, std::size_t index) noexcept {
        // 越界检测
        if (index > 0 && (index + DataLen) < (Capacity - 1)) {
            if constexpr (std::is_trivially_copyable_v<T>) {
                std::memcpy(_buffer.data() + index, &data, DataLen);
                return true;
            }
        }
        return std::nullopt;
    }

    /**
     * @brief Self-define data reader
     * 自定义解析数据
     * @tparam T Data type
     * @tparam DataLen Data length, defaults to sizeof(T)
     * @param data 解析后的数据输出参数
     * @param index 源数据位置
     * @return std::optional<bool> 成功返回true，失败返回std::nullopt
     */
    template<typename T, std::size_t DataLen = sizeof(T)>
    std::optional<bool> unload_data(T& data, std::size_t index) const noexcept {
        // 越界检测
        if (index > 0 && (index + DataLen) < (Capacity - 1)) {
            if constexpr (std::is_trivially_copyable_v<T>) {
                std::memcpy(&data, _buffer.data() + index, DataLen);
                return true;
            }
        }
        return std::nullopt;
    }

    /**
     * @brief Validates if the packet has valid head and tail bytes
     * 验证数据包是否有有效的头尾字节
     * @return true if valid, false otherwise
     */
    [[nodiscard]] bool is_valid() const noexcept {
        return _buffer[0] == HEAD_BYTE && _buffer[Capacity - 1] == TAIL_BYTE;
    }

private:
    // 数据包缓存buffer
    std::array<uint8_t, Capacity> _buffer;
};

// 常用的固定大小包类型别名
using FixedPacket16 = FixedPacket<16>;
using FixedPacket32 = FixedPacket<32>;
using FixedPacket64 = FixedPacket<64>;

} // namespace serial
#endif //FIXED_PACKET_HPP
