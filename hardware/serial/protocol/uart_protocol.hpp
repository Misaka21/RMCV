//
// Created by misaka21 on 25-3-1.
//

#ifndef UART_PROTOCOL_HPP
#define UART_PROTOCOL_HPP

// C system headers

// C++ system headers
#include <string>

// Third-party library headers

// Project headers
#include "protocol_interface.hpp"

class UartProtocol: public ProtocolInterface {
public:
    UartProtocol(
        std::string_view device_path,
        int speed = 115200,
        int flow_ctrl = 0,
        int databits = 8,
        int stopbits = 1,
        int parity = 'N'
    ):
        _device_path(device_path),
        _speed(speed),
        _flow_ctrl(flow_ctrl),
        _databits(databits),
        _stopbits(stopbits),
        _parity(parity) {}
    ~UartProtocol() override = default;

    [[nodiscard]] bool open() override;
    void close() noexcept override;
    [[nodiscard]] bool is_open() const noexcept override;

    [[nodiscard]] int read(std::byte* buffer, std::size_t len) noexcept override;
    [[nodiscard]] int write(const std::byte* buffer, std::size_t len) noexcept override;

    [[nodiscard]] std::string error_message() const override {
        return _error_message;
    }

private:
    bool set_param(
        int speed = 115200,
        int flow_ctrl = 0,
        int databits = 0,
        int stopbits = 1,
        int parity = 'N'
    );
    // 设备文件描述符
    int _fd { -1 };
    // 设备状态
    bool _is_open { false };
    std::string _error_message;
    // 设备参数
    std::string _device_path;
    int _speed;
    int _flow_ctrl;
    int _databits;
    int _stopbits;
    int _parity;
};

#endif //UART_PROTOCOL_HPP
