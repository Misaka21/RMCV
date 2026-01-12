//
// Created by Claude on 25-10-19.
// 借鉴 librmcs (https://github.com/Alliance-EC/librmcs) 的设计
//

// Source file corresponding header
#include "usb_bulk_protocol.hpp"

// C system headers

// C++ system headers
#include <cstring>
#include <iomanip>
#include <sstream>

// Third-party library headers

// Project headers
#include "plugin/debug/logger.hpp"

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
    _disconnected = false;

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

    debug::print(
        debug::PrintMode::INFO, "UsbBulk",
        "Connected: VID=0x{:04x} PID=0x{:04x} SN={}",
        _descriptor.vendor_id, _connected_product_id,
        _actual_serial.empty() ? "(none)" : _actual_serial
    );

    return true;
}

void UsbBulkProtocol::close() noexcept {
    if (_is_open) {
        release_device();
        _is_open = false;
        debug::print(debug::PrintMode::INFO, "UsbBulk", "Disconnected");
    }
}

bool UsbBulkProtocol::is_open() const noexcept {
    return _is_open && !_disconnected;
}

int UsbBulkProtocol::read(std::byte* buffer, std::size_t len) noexcept {
    if (!_is_open || !_handle) {
        _error_message = "Device not open";
        return -1;
    }

    if (_disconnected) {
        _error_message = "Device disconnected";
        return -1;
    }

    if (!buffer || len == 0) {
        _error_message = "Invalid buffer or length";
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
        // 超时不是错误，返回0
        return 0;
    } else {
        handle_transfer_error(result);
        return -1;
    }
}

int UsbBulkProtocol::write(const std::byte* buffer, std::size_t len) noexcept {
    if (!_is_open || !_handle) {
        _error_message = "Device not open";
        return -1;
    }

    if (_disconnected) {
        _error_message = "Device disconnected";
        return -1;
    }

    if (!buffer || len == 0) {
        _error_message = "Invalid buffer or length";
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
        handle_transfer_error(result);
        return -1;
    }
}

bool UsbBulkProtocol::handle_transfer_error(int result) noexcept {
    _error_message = get_libusb_error(result);

    // 检测断线错误
    if (result == LIBUSB_ERROR_NO_DEVICE ||
        result == LIBUSB_ERROR_IO ||
        result == LIBUSB_ERROR_PIPE) {
        _disconnected = true;
        debug::print(debug::PrintMode::ERROR, "UsbBulk", "Device disconnected: {}", _error_message);
        return false;
    }

    debug::print(debug::PrintMode::WARNING, "UsbBulk", "Transfer error: {}", _error_message);
    return true;  // 可重试
}

std::vector<UsbBulkProtocol::DeviceInfo> UsbBulkProtocol::list_devices(
    uint16_t vendor_id,
    std::optional<uint16_t> product_id
) {
    std::vector<DeviceInfo> devices;
    libusb_context* ctx = nullptr;
    libusb_device** dev_list = nullptr;

    int result = libusb_init(&ctx);
    if (result != LIBUSB_SUCCESS) {
        return devices;
    }

    ssize_t device_count = libusb_get_device_list(ctx, &dev_list);
    if (device_count < 0) {
        libusb_exit(ctx);
        return devices;
    }

    for (ssize_t i = 0; i < device_count; ++i) {
        libusb_device* device = dev_list[i];
        struct libusb_device_descriptor desc;

        result = libusb_get_device_descriptor(device, &desc);
        if (result != LIBUSB_SUCCESS) {
            continue;
        }

        // 检查厂商ID
        if (desc.idVendor != vendor_id) {
            continue;
        }

        // 检查产品ID (如果指定了的话)
        if (product_id.has_value() && desc.idProduct != product_id.value()) {
            continue;
        }

        DeviceInfo info;
        info.vendor_id = desc.idVendor;
        info.product_id = desc.idProduct;

        // 尝试获取字符串描述符
        libusb_device_handle* handle = nullptr;
        result = libusb_open(device, &handle);
        if (result == LIBUSB_SUCCESS && handle) {
            unsigned char buffer[256] = {0};

            // 序列号
            if (desc.iSerialNumber) {
                result = libusb_get_string_descriptor_ascii(
                    handle, desc.iSerialNumber, buffer, sizeof(buffer));
                if (result > 0) {
                    info.serial_number = std::string(reinterpret_cast<char*>(buffer), result);
                }
            }

            // 制造商
            if (desc.iManufacturer) {
                result = libusb_get_string_descriptor_ascii(
                    handle, desc.iManufacturer, buffer, sizeof(buffer));
                if (result > 0) {
                    info.manufacturer = std::string(reinterpret_cast<char*>(buffer), result);
                }
            }

            // 产品名
            if (desc.iProduct) {
                result = libusb_get_string_descriptor_ascii(
                    handle, desc.iProduct, buffer, sizeof(buffer));
                if (result > 0) {
                    info.product = std::string(reinterpret_cast<char*>(buffer), result);
                }
            }

            libusb_close(handle);
        }

        devices.push_back(info);
    }

    libusb_free_device_list(dev_list, 1);
    libusb_exit(ctx);

    return devices;
}

std::string UsbBulkProtocol::get_device_info() const {
    if (!_is_open || !_handle) {
        return "Device not connected";
    }

    std::ostringstream info;
    info << "USB Device Info:\n";
    info << "  Vendor ID: 0x" << std::hex << std::setw(4) << std::setfill('0')
         << _descriptor.vendor_id << std::dec << "\n";
    info << "  Product ID: 0x" << std::hex << std::setw(4) << std::setfill('0')
         << _connected_product_id << std::dec << "\n";
    info << "  Interface: " << static_cast<int>(_descriptor.interface_number) << "\n";
    info << "  Bulk IN: 0x" << std::hex << std::setw(2) << std::setfill('0')
         << static_cast<int>(_descriptor.bulk_in_endpoint) << std::dec << "\n";
    info << "  Bulk OUT: 0x" << std::hex << std::setw(2) << std::setfill('0')
         << static_cast<int>(_descriptor.bulk_out_endpoint) << std::dec << "\n";

    if (!_actual_serial.empty()) {
        info << "  Serial: " << _actual_serial << "\n";
    }

    return info.str();
}

void UsbBulkProtocol::set_read_timeout(int timeout_ms) {
    _read_timeout_ms = timeout_ms;
}

void UsbBulkProtocol::set_write_timeout(int timeout_ms) {
    _write_timeout_ms = timeout_ms;
}

std::shared_ptr<UsbBulkProtocol> UsbBulkProtocol::create_from_config(
    const std::string& vendor_id_str,
    const std::string& product_id_str,
    const std::string& serial_number,
    int interface_number,
    const std::string& bulk_in_endpoint_str,
    const std::string& bulk_out_endpoint_str,
    int timeout_ms
) {
    // 解析 vendor_id
    auto vendor_id = parse_hex_string(vendor_id_str);
    if (!vendor_id.has_value()) {
        debug::print(debug::PrintMode::ERROR, "UsbBulk",
            "Invalid vendor_id: {}", vendor_id_str);
        return nullptr;
    }

    // 解析 product_id (可选)
    std::optional<uint16_t> product_id = std::nullopt;
    if (!product_id_str.empty()) {
        product_id = parse_hex_string(product_id_str);
        if (!product_id.has_value()) {
            debug::print(debug::PrintMode::ERROR, "UsbBulk",
                "Invalid product_id: {}", product_id_str);
            return nullptr;
        }
    }

    // 解析端点
    auto bulk_in = parse_hex_string(bulk_in_endpoint_str);
    auto bulk_out = parse_hex_string(bulk_out_endpoint_str);
    if (!bulk_in.has_value() || !bulk_out.has_value()) {
        debug::print(debug::PrintMode::ERROR, "UsbBulk",
            "Invalid endpoint: IN={} OUT={}", bulk_in_endpoint_str, bulk_out_endpoint_str);
        return nullptr;
    }

    UsbDeviceDescriptor descriptor {
        .vendor_id = vendor_id.value(),
        .product_id = product_id,
        .interface_number = static_cast<uint8_t>(interface_number),
        .bulk_in_endpoint = static_cast<uint8_t>(bulk_in.value()),
        .bulk_out_endpoint = static_cast<uint8_t>(bulk_out.value()),
        .timeout_ms = timeout_ms
    };

    return std::make_shared<UsbBulkProtocol>(descriptor, serial_number);
}

bool UsbBulkProtocol::init_libusb() {
    if (_ctx) {
        return true;
    }

    int result = libusb_init(&_ctx);
    if (result != LIBUSB_SUCCESS) {
        _error_message = "libusb init failed: " + get_libusb_error(result);
        return false;
    }

#ifdef DEBUG
    libusb_set_option(_ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_WARNING);
#endif

    return true;
}

bool UsbBulkProtocol::find_and_open_device() {
    libusb_device** dev_list = nullptr;
    ssize_t device_count = libusb_get_device_list(_ctx, &dev_list);

    if (device_count < 0) {
        _error_message = "Failed to get device list: " + get_libusb_error(static_cast<int>(device_count));
        return false;
    }

    // 统计匹配的设备
    int valid_device_count = 0;
    libusb_device* selected_device = nullptr;
    uint16_t selected_product_id = 0;
    std::string selected_serial;

    for (ssize_t i = 0; i < device_count; ++i) {
        libusb_device* device = dev_list[i];
        struct libusb_device_descriptor desc;

        int result = libusb_get_device_descriptor(device, &desc);
        if (result != LIBUSB_SUCCESS) {
            continue;
        }

        // 检查厂商ID
        if (desc.idVendor != _descriptor.vendor_id) {
            continue;
        }

        // 检查产品ID (如果指定了的话)
        if (_descriptor.product_id.has_value() &&
            desc.idProduct != _descriptor.product_id.value()) {
            continue;
        }

        // 获取序列号
        std::string actual_serial;
        libusb_device_handle* temp_handle = nullptr;
        result = libusb_open(device, &temp_handle);
        if (result == LIBUSB_SUCCESS && temp_handle) {
            unsigned char serial_buffer[256] = {0};
            if (desc.iSerialNumber) {
                int serial_result = libusb_get_string_descriptor_ascii(
                    temp_handle, desc.iSerialNumber, serial_buffer, sizeof(serial_buffer));
                if (serial_result > 0) {
                    actual_serial = std::string(reinterpret_cast<char*>(serial_buffer), serial_result);
                }
            }
            libusb_close(temp_handle);

            // 如果指定了序列号，需要匹配
            if (!_serial_number.empty() && actual_serial != _serial_number) {
                continue;
            }

            valid_device_count++;
            selected_device = device;
            selected_product_id = desc.idProduct;
            selected_serial = actual_serial;
        }
    }

    // 检查匹配结果
    if (valid_device_count == 0) {
        std::ostringstream msg;
        msg << "No device found with VID=0x" << std::hex << _descriptor.vendor_id;
        if (_descriptor.product_id.has_value()) {
            msg << " PID=0x" << _descriptor.product_id.value();
        }
        if (!_serial_number.empty()) {
            msg << " SN=" << _serial_number;
        }
        _error_message = msg.str();

        // 列出找到的同厂商设备
        auto devices = list_devices(_descriptor.vendor_id);
        if (!devices.empty()) {
            debug::print(debug::PrintMode::INFO, "UsbBulk", "Available devices with VID=0x{:04x}:", _descriptor.vendor_id);
            for (const auto& dev : devices) {
                debug::print(debug::PrintMode::INFO, "UsbBulk",
                    "  PID=0x{:04x} SN={} ({})",
                    dev.product_id,
                    dev.serial_number.empty() ? "(none)" : dev.serial_number,
                    dev.product.empty() ? "unknown" : dev.product);
            }
        }

        libusb_free_device_list(dev_list, 1);
        return false;
    }

    if (valid_device_count > 1) {
        debug::print(debug::PrintMode::WARNING, "UsbBulk",
            "Multiple devices ({}) found, using first match. Consider specifying serial_number.",
            valid_device_count);
    }

    // 打开选中的设备
    int result = libusb_open(selected_device, &_handle);
    libusb_free_device_list(dev_list, 1);

    if (result != LIBUSB_SUCCESS) {
        _error_message = "Failed to open device: " + get_libusb_error(result);
        return false;
    }

    _connected_product_id = selected_product_id;
    _actual_serial = selected_serial;

    return true;
}

bool UsbBulkProtocol::configure_device() {
    if (!_handle) {
        return false;
    }

    int result;

    // Linux: 自动卸载内核驱动
#ifdef __linux__
    result = libusb_set_auto_detach_kernel_driver(_handle, 1);
    if (result != LIBUSB_SUCCESS && result != LIBUSB_ERROR_NOT_SUPPORTED) {
        debug::print(debug::PrintMode::WARNING, "UsbBulk",
            "Failed to set auto detach kernel driver: {}", get_libusb_error(result));
    }
#else
    // 非Linux: 手动检查并卸载
    if (libusb_kernel_driver_active(_handle, _descriptor.interface_number) == 1) {
        result = libusb_detach_kernel_driver(_handle, _descriptor.interface_number);
        if (result != LIBUSB_SUCCESS) {
            _error_message = "Failed to detach kernel driver: " + get_libusb_error(result);
            return false;
        }
    }
#endif

    // 声明接口
    result = libusb_claim_interface(_handle, _descriptor.interface_number);
    if (result != LIBUSB_SUCCESS) {
        _error_message = "Failed to claim interface: " + get_libusb_error(result);
        return false;
    }

    return true;
}

void UsbBulkProtocol::release_device() {
    if (_handle) {
        libusb_release_interface(_handle, _descriptor.interface_number);

#ifndef __linux__
        // 非Linux: 尝试重新附加内核驱动
        libusb_attach_kernel_driver(_handle, _descriptor.interface_number);
#endif

        libusb_close(_handle);
        _handle = nullptr;
    }
}

std::string UsbBulkProtocol::get_libusb_error(int error_code) {
    switch (error_code) {
        case LIBUSB_SUCCESS: return "Success";
        case LIBUSB_ERROR_IO: return "I/O error";
        case LIBUSB_ERROR_INVALID_PARAM: return "Invalid parameter";
        case LIBUSB_ERROR_ACCESS: return "Access denied";
        case LIBUSB_ERROR_NO_DEVICE: return "No such device (disconnected?)";
        case LIBUSB_ERROR_NOT_FOUND: return "Entity not found";
        case LIBUSB_ERROR_BUSY: return "Resource busy";
        case LIBUSB_ERROR_TIMEOUT: return "Operation timed out";
        case LIBUSB_ERROR_OVERFLOW: return "Overflow";
        case LIBUSB_ERROR_PIPE: return "Pipe error";
        case LIBUSB_ERROR_INTERRUPTED: return "System call interrupted";
        case LIBUSB_ERROR_NO_MEM: return "Insufficient memory";
        case LIBUSB_ERROR_NOT_SUPPORTED: return "Operation not supported";
        case LIBUSB_ERROR_OTHER: return "Other error";
        default: return "Unknown error: " + std::to_string(error_code);
    }
}

void UsbBulkProtocol::cleanup_libusb() {
    if (_ctx) {
        libusb_exit(_ctx);
        _ctx = nullptr;
    }
}

std::optional<uint16_t> UsbBulkProtocol::parse_hex_string(const std::string& hex_str) {
    if (hex_str.empty()) {
        return std::nullopt;
    }

    try {
        // 支持 "0x1234", "0X1234", "1234" 格式
        size_t pos = 0;
        unsigned long value = std::stoul(hex_str, &pos, 0);  // base=0 自动检测进制
        if (pos != hex_str.length()) {
            return std::nullopt;  // 未完全解析
        }
        if (value > 0xFFFF) {
            return std::nullopt;  // 超出范围
        }
        return static_cast<uint16_t>(value);
    } catch (...) {
        return std::nullopt;
    }
}
