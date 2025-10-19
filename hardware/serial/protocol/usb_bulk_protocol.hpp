//
// Created by Claude on 25-10-19.
//

#ifndef USB_BULK_PROTOCOL_HPP
#define USB_BULK_PROTOCOL_HPP

// C system headers

// C++ system headers
#include <memory>
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
        uint16_t product_id;
        uint8_t interface_number;
        uint8_t bulk_in_endpoint;
        uint8_t bulk_out_endpoint;
        int timeout_ms;
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
     * @brief 列出所有可用的USB设备
     * @param vendor_id 厂商ID
     * @param product_id 产品ID
     * @return 设备序列号列表
     */
    [[nodiscard]] static std::vector<std::string> list_available_devices(
        uint16_t vendor_id,
        uint16_t product_id
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
    std::string get_libusb_error(int error_code) const;

    /**
     * @brief 清理libusb库
     */
    void cleanup_libusb();

private:
    UsbDeviceDescriptor _descriptor;
    std::string _serial_number;

    libusb_context* _ctx { nullptr };
    libusb_device_handle* _handle { nullptr };

    bool _is_open { false };
    std::string _error_message;

    int _read_timeout_ms;
    int _write_timeout_ms;

    static bool _libusb_initialized;
};

#endif // USB_BULK_PROTOCOL_HPP