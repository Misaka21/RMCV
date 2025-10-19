//
// Created by Claude on 25-10-19.
//

#include "usb_bulk_protocol.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>

bool UsbBulkProtocol::_libusb_initialized = false;

UsbBulkProtocol::UsbBulkProtocol(
    const UsbDeviceDescriptor& descriptor,
    std::string_view serial_number
) : _descriptor(descriptor),
    _serial_number(serial_number),
    _read_timeout_ms(descriptor.timeout_ms),
    _write_timeout_ms(descriptor.timeout_ms) {
}

UsbBulkProtocol::~UsbBulkProtocol() {
    close();
    cleanup_libusb();
}

bool UsbBulkProtocol::open() {
    if (_is_open) {
        return true;
    }

    _error_message.clear();

    // 初始化libusb
    if (!init_libusb()) {
        return false;
    }

    // 查找并打开设备
    if (!find_and_open_device()) {
        return false;
    }

    // 配置设备
    if (!configure_device()) {
        release_device();
        return false;
    }

    _is_open = true;
    return true;
}

void UsbBulkProtocol::close() noexcept {
    if (_is_open) {
        release_device();
        _is_open = false;
    }
}

bool UsbBulkProtocol::is_open() const noexcept {
    return _is_open;
}

int UsbBulkProtocol::read(std::byte* buffer, std::size_t len) noexcept {
    if (!_is_open || !_handle) {
        _error_message = "设备未打开";
        return -1;
    }

    if (!buffer || len == 0) {
        _error_message = "无效的缓冲区或长度";
        return -1;
    }

    int actual_length = 0;
    int result = libusb_bulk_transfer(
        _handle,
        _descriptor.bulk_in_endpoint,
        reinterpret_cast<unsigned char*>(buffer),
        static_cast<int>(len),
        &actual_length,
        _read_timeout_ms
    );

    if (result == LIBUSB_SUCCESS) {
        return actual_length;
    } else if (result == LIBUSB_ERROR_TIMEOUT) {
        _error_message = "读取超时";
        return 0;
    } else {
        _error_message = "读取失败: " + get_libusb_error(result);
        return -1;
    }
}

int UsbBulkProtocol::write(const std::byte* buffer, std::size_t len) noexcept {
    if (!_is_open || !_handle) {
        _error_message = "设备未打开";
        return -1;
    }

    if (!buffer || len == 0) {
        _error_message = "无效的缓冲区或长度";
        return -1;
    }

    int actual_length = 0;
    int result = libusb_bulk_transfer(
        _handle,
        _descriptor.bulk_out_endpoint,
        const_cast<unsigned char*>(reinterpret_cast<const unsigned char*>(buffer)),
        static_cast<int>(len),
        &actual_length,
        _write_timeout_ms
    );

    if (result == LIBUSB_SUCCESS) {
        return actual_length;
    } else {
        _error_message = "写入失败: " + get_libusb_error(result);
        return -1;
    }
}

std::vector<std::string> UsbBulkProtocol::list_available_devices(
    uint16_t vendor_id,
    uint16_t product_id
) {
    std::vector<std::string> devices;
    libusb_context* ctx = nullptr;
    libusb_device** dev_list = nullptr;
    ssize_t device_count = 0;

    // 初始化libusb
    int result = libusb_init(&ctx);
    if (result != LIBUSB_SUCCESS) {
        return devices;
    }

    // 获取设备列表
    device_count = libusb_get_device_list(ctx, &dev_list);
    if (device_count < 0) {
        libusb_exit(ctx);
        return devices;
    }

    // 遍历设备
    for (ssize_t i = 0; i < device_count; ++i) {
        libusb_device* device = dev_list[i];
        struct libusb_device_descriptor desc;

        result = libusb_get_device_descriptor(device, &desc);
        if (result != LIBUSB_SUCCESS) {
            continue;
        }

        // 检查厂商ID和产品ID
        if (desc.idVendor == vendor_id && desc.idProduct == product_id) {
            libusb_device_handle* handle = nullptr;
            result = libusb_open(device, &handle);

            if (result == LIBUSB_SUCCESS && handle) {
                unsigned char serial_buffer[256] = {0};
                result = libusb_get_string_descriptor_ascii(
                    handle,
                    desc.iSerialNumber,
                    serial_buffer,
                    sizeof(serial_buffer)
                );

                if (result > 0) {
                    std::string serial(reinterpret_cast<char*>(serial_buffer), result);
                    devices.push_back(serial);
                } else {
                    // 如果没有序列号，使用位置作为标识
                    devices.push_back("Device_" + std::to_string(i));
                }

                libusb_close(handle);
            }
        }
    }

    libusb_free_device_list(dev_list, 1);
    libusb_exit(ctx);

    return devices;
}

std::string UsbBulkProtocol::get_device_info() const {
    if (!_is_open || !_handle) {
        return "设备未连接";
    }

    std::ostringstream info;
    info << "USB设备信息:\n";
    info << "  厂商ID: 0x" << std::hex << std::setw(4) << std::setfill('0')
         << _descriptor.vendor_id << std::dec << "\n";
    info << "  产品ID: 0x" << std::hex << std::setw(4) << std::setfill('0')
         << _descriptor.product_id << std::dec << "\n";
    info << "  接口号: " << static_cast<int>(_descriptor.interface_number) << "\n";
    info << "  Bulk IN端点: 0x" << std::hex << std::setw(2) << std::setfill('0')
         << static_cast<int>(_descriptor.bulk_in_endpoint) << std::dec << "\n";
    info << "  Bulk OUT端点: 0x" << std::hex << std::setw(2) << std::setfill('0')
         << static_cast<int>(_descriptor.bulk_out_endpoint) << std::dec << "\n";

    if (!_serial_number.empty()) {
        info << "  序列号: " << _serial_number << "\n";
    }

    return info.str();
}

void UsbBulkProtocol::set_read_timeout(int timeout_ms) {
    _read_timeout_ms = timeout_ms;
}

void UsbBulkProtocol::set_write_timeout(int timeout_ms) {
    _write_timeout_ms = timeout_ms;
}

bool UsbBulkProtocol::init_libusb() {
    if (_libusb_initialized) {
        return true;
    }

    int result = libusb_init(&_ctx);
    if (result != LIBUSB_SUCCESS) {
        _error_message = "libusb初始化失败: " + get_libusb_error(result);
        return false;
    }

    // 设置调试级别（可选）
    #ifdef DEBUG
    libusb_set_option(_ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_DEBUG);
    #endif

    _libusb_initialized = true;
    return true;
}

bool UsbBulkProtocol::find_and_open_device() {
    libusb_device** dev_list = nullptr;
    ssize_t device_count = libusb_get_device_list(_ctx, &dev_list);

    if (device_count < 0) {
        _error_message = "获取设备列表失败: " + get_libusb_error(static_cast<int>(device_count));
        return false;
    }

    bool found = false;
    std::string actual_serial;

    // 遍历设备查找匹配的设备
    for (ssize_t i = 0; i < device_count; ++i) {
        libusb_device* device = dev_list[i];
        struct libusb_device_descriptor desc;

        int result = libusb_get_device_descriptor(device, &desc);
        if (result != LIBUSB_SUCCESS) {
            continue;
        }

        // 检查厂商ID和产品ID
        if (desc.idVendor == _descriptor.vendor_id &&
            desc.idProduct == _descriptor.product_id) {

            // 尝试获取设备的序列号
            libusb_device_handle* temp_handle = nullptr;
            result = libusb_open(device, &temp_handle);

            if (result == LIBUSB_SUCCESS && temp_handle) {
                unsigned char serial_buffer[256] = {0};
                int serial_result = libusb_get_string_descriptor_ascii(
                    temp_handle,
                    desc.iSerialNumber,
                    serial_buffer,
                    sizeof(serial_buffer)
                );

                if (serial_result > 0) {
                    actual_serial = std::string(reinterpret_cast<char*>(serial_buffer), serial_result);
                } else {
                    // 如果没有序列号，使用位置作为标识
                    actual_serial = "Device_" + std::to_string(i);
                }

                libusb_close(temp_handle);

                // 如果指定了序列号，需要验证
                if (!_serial_number.empty() && actual_serial != _serial_number) {
                    continue; // 序列号不匹配，继续查找
                }

                // 打开设备
                result = libusb_open(device, &_handle);
                if (result == LIBUSB_SUCCESS) {
                    found = true;
                    // 保存实际使用的序列号
                    _serial_number = actual_serial;
                    break;
                } else {
                    _error_message = "无法打开设备: " + get_libusb_error(result);
                }
            }
        }
    }

    libusb_free_device_list(dev_list, 1);

    if (!found || !_handle) {
        _error_message = "未找到指定的USB设备 (VID: 0x" +
                        std::to_string(_descriptor.vendor_id) +
                        ", PID: 0x" + std::to_string(_descriptor.product_id) + ")";
        return false;
    }

    return true;
}

bool UsbBulkProtocol::configure_device() {
    if (!_handle) {
        return false;
    }

    // 检查内核驱动是否已加载，如果是则卸载
    if (libusb_kernel_driver_active(_handle, _descriptor.interface_number) == 1) {
        int result = libusb_detach_kernel_driver(_handle, _descriptor.interface_number);
        if (result != LIBUSB_SUCCESS) {
            _error_message = "无法卸载内核驱动: " + get_libusb_error(result);
            return false;
        }
    }

    // 声明接口
    int result = libusb_claim_interface(_handle, _descriptor.interface_number);
    if (result != LIBUSB_SUCCESS) {
        _error_message = "无法声明接口: " + get_libusb_error(result);
        return false;
    }

    return true;
}

void UsbBulkProtocol::release_device() {
    if (_handle) {
        // 释放接口
        libusb_release_interface(_handle, _descriptor.interface_number);

        // 重新附加内核驱动（如果之前被卸载了）
        libusb_attach_kernel_driver(_handle, _descriptor.interface_number);

        // 关闭设备
        libusb_close(_handle);
        _handle = nullptr;
    }
}

std::string UsbBulkProtocol::get_libusb_error(int error_code) const {
    switch (error_code) {
        case LIBUSB_SUCCESS: return "成功";
        case LIBUSB_ERROR_IO: return "输入输出错误";
        case LIBUSB_ERROR_INVALID_PARAM: return "无效参数";
        case LIBUSB_ERROR_ACCESS: return "访问被拒绝";
        case LIBUSB_ERROR_NO_DEVICE: return "设备不存在";
        case LIBUSB_ERROR_NOT_FOUND: return "未找到";
        case LIBUSB_ERROR_BUSY: return "设备忙";
        case LIBUSB_ERROR_TIMEOUT: return "操作超时";
        case LIBUSB_ERROR_OVERFLOW: return "溢出";
        case LIBUSB_ERROR_PIPE: return "管道错误";
        case LIBUSB_ERROR_INTERRUPTED: return "操作被中断";
        case LIBUSB_ERROR_NO_MEM: return "内存不足";
        case LIBUSB_ERROR_NOT_SUPPORTED: return "不支持的操作";
        case LIBUSB_ERROR_OTHER: return "其他错误";
        default: return "未知错误: " + std::to_string(error_code);
    }
}

void UsbBulkProtocol::cleanup_libusb() {
    if (_libusb_initialized && _ctx) {
        libusb_exit(_ctx);
        _ctx = nullptr;
        _libusb_initialized = false;
    }
}