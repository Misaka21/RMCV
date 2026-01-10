//
// Created by Claude on 25-10-19.
// 借鉴 librmcs (https://github.com/Alliance-EC/librmcs) 的设计
//

#ifndef USB_BULK_PROTOCOL_HPP
#define USB_BULK_PROTOCOL_HPP

// C system headers

// C++ system headers
#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

// Third-party library headers
#include <libusb-1.0/libusb.h>

// Project headers
#include "protocol_interface.hpp"

class UsbBulkProtocol: public ProtocolInterface {
public:
    /**
     * @brief USB设备描述符结构体
     */
    struct UsbDeviceDescriptor {
        uint16_t vendor_id;
        std::optional<uint16_t> product_id;  // 空表示匹配该厂商所有设备
        uint8_t interface_number;
        uint8_t bulk_in_endpoint;
        uint8_t bulk_out_endpoint;
        int timeout_ms;
    };

    /**
     * @brief 设备信息结构体 (用于枚举设备)
     */
    struct DeviceInfo {
        uint16_t vendor_id;
        uint16_t product_id;
        std::string serial_number;
        std::string manufacturer;
        std::string product;
    };

    /**
     * @brief 构造函数
     * @param descriptor USB设备描述符
     * @param serial_number 可选的设备序列号，用于区分同名设备。
     *                      如果为空，将自动连接第一个匹配VID/PID的设备
     */
    UsbBulkProtocol(
        const UsbDeviceDescriptor& descriptor,
        std::string_view serial_number = ""
    );

    ~UsbBulkProtocol() override;

    // 禁止拷贝
    UsbBulkProtocol(const UsbBulkProtocol&) = delete;
    UsbBulkProtocol& operator=(const UsbBulkProtocol&) = delete;

    // ProtocolInterface 接口实现
    [[nodiscard]] bool open() override;
    void close() noexcept override;
    [[nodiscard]] bool is_open() const noexcept override;

    [[nodiscard]] int read(std::byte* buffer, std::size_t len) noexcept override;
    [[nodiscard]] int write(const std::byte* buffer, std::size_t len) noexcept override;

    [[nodiscard]] std::string error_message() const override {
        return _error_message;
    }

    // USB特定的功能

    /**
     * @brief 列出所有匹配的USB设备
     * @param vendor_id 厂商ID
     * @param product_id 产品ID，空表示匹配所有
     * @return 设备信息列表
     */
    [[nodiscard]] static std::vector<DeviceInfo> list_devices(
        uint16_t vendor_id,
        std::optional<uint16_t> product_id = std::nullopt
    );

    /**
     * @brief 获取当前连接的设备信息
     * @return 设备信息字符串，如果未连接则返回错误信息
     */
    [[nodiscard]] std::string get_device_info() const;

    /**
     * @brief 设置读取超时时间
     * @param timeout_ms 超时时间（毫秒）
     */
    void set_read_timeout(int timeout_ms);

    /**
     * @brief 设置写入超时时间
     * @param timeout_ms 超时时间（毫秒）
     */
    void set_write_timeout(int timeout_ms);

    /**
     * @brief 检查设备是否断开
     * @return true 表示设备已断开
     */
    [[nodiscard]] bool is_disconnected() const noexcept { return _disconnected; }

    /**
     * @brief 从配置创建 UsbBulkProtocol (工厂方法)
     * @param vendor_id_str 厂商ID字符串 (如 "0xa11c")
     * @param product_id_str 产品ID字符串, 空表示匹配所有
     * @param serial_number 设备序列号
     * @param interface_number 接口号
     * @param bulk_in_endpoint_str IN端点字符串 (如 "0x81")
     * @param bulk_out_endpoint_str OUT端点字符串 (如 "0x01")
     * @param timeout_ms 超时时间
     * @return 成功返回 UsbBulkProtocol 指针，失败返回 nullptr
     */
    [[nodiscard]] static std::shared_ptr<UsbBulkProtocol> create_from_config(
        const std::string& vendor_id_str,
        const std::string& product_id_str,
        const std::string& serial_number,
        int interface_number,
        const std::string& bulk_in_endpoint_str,
        const std::string& bulk_out_endpoint_str,
        int timeout_ms
    );

private:
    /**
     * @brief 初始化libusb库
     * @return 成功返回true
     */
    bool init_libusb();

    /**
     * @brief 查找并打开指定的USB设备
     * @return 成功返回true
     */
    bool find_and_open_device();

    /**
     * @brief 配置USB接口和端点
     * @return 成功返回true
     */
    bool configure_device();

    /**
     * @brief 释放USB设备资源
     */
    void release_device();

    /**
     * @brief 获取libusb错误信息的字符串描述
     * @param error_code libusb错误码
     * @return 错误信息字符串
     */
    static std::string get_libusb_error(int error_code);

    /**
     * @brief 清理libusb库
     */
    void cleanup_libusb();

    /**
     * @brief 解析16进制字符串
     * @param hex_str 16进制字符串 (如 "0x81" 或 "81")
     * @return 解析结果，失败返回 nullopt
     */
    static std::optional<uint16_t> parse_hex_string(const std::string& hex_str);

    /**
     * @brief 处理传输错误，检测断线
     * @param result libusb 返回值
     * @return 是否应该重试
     */
    bool handle_transfer_error(int result) noexcept;

private:
    UsbDeviceDescriptor _descriptor;
    std::string _serial_number;
    std::string _actual_serial;  // 实际连接的设备序列号

    libusb_context* _ctx { nullptr };
    libusb_device_handle* _handle { nullptr };

    bool _is_open { false };
    std::atomic<bool> _disconnected { false }; 
    std::string _error_message;

    int _read_timeout_ms;
    int _write_timeout_ms;

    // 连接的设备信息
    uint16_t _connected_product_id { 0 };
};

#endif // USB_BULK_PROTOCOL_HPP