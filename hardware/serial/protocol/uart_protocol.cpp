#include "uart_protocol.hpp"

#include <algorithm>
#include <array>
#include <cstring> // 用于 std::strerror
#include <errno.h> /*错误号定义*/
#include <fcntl.h> /*文件控制定义*/
#include <stdio.h> /*标准输入输出定义*/
#include <stdlib.h> /*标准函数库定义*/
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <system_error> // 用于更完善的错误处理
#include <termios.h> /*PPSIX 终端控制定义*/
#include <termios.h>
#include <unistd.h> /*Unix 标准函数定义*/

bool UartProtocol::set_param(int speed, int flow_ctrl, int databits, int stopbits, int parity) {
    // 设置串口数据帧格式
    constexpr std::array<std::pair<int, int>, 14> baud_rates = {
        { //{B1152000, 1152000}, {B1000000, 1000000}, {B921600, 921600},
          //{B576000, 576000},   {B500000, 500000},   {B460800, 460800},
          { B230400, 230400 },
          { B115200, 115200 },
          { B19200, 19200 },
          { B9600, 9600 },
          { B4800, 4800 },
          { B2400, 2400 },
          { B1200, 1200 },
          { B300, 300 } }
    };

    termios options;
    if (tcgetattr(_fd, &options) != 0) {
        _error_message = "tcgetattr failed: " + std::string(strerror(errno));
        return false;
    }

    // 设置串口输入波特率和输出波特率
    bool baud_found = false;
    for (const auto& [sys_baud, user_baud]: baud_rates) {
        if (speed == user_baud) {
            cfsetispeed(&options, sys_baud);
            cfsetospeed(&options, sys_baud);
            baud_found = true;
            break;
        }
    }
    if (!baud_found) {
        _error_message = "Unsupported baud rate: " + std::to_string(speed);
        return false;
    }

    enum class FlowControl { None, Hardware, Software };
    const auto flow = static_cast<FlowControl>(flow_ctrl);

    // 修改控制模式，保证程序不会占用串口
    // 修改控制模式，使得能够从串口中读取输入数据
    options.c_cflag |= (CLOCAL | CREAD);

    switch (flow) {
        case FlowControl::None: // 不使用流控制
            options.c_cflag &= ~CRTSCTS;
            break;
        case FlowControl::Hardware: // 使用硬件流控制
            options.c_cflag |= CRTSCTS;
            break;
        case FlowControl::Software: // 使用软件流控制
            options.c_iflag |= (IXON | IXOFF | IXANY);
            break;
        default:
            _error_message = "Invalid flow control: " + std::to_string(flow_ctrl);
            return false;
    }

    // 数据位设置
    constexpr std::array<int, 4> valid_databits = { 5, 6, 7, 8 };
    if (std::find(valid_databits.begin(), valid_databits.end(), databits) == valid_databits.end()) {
        _error_message = "Invalid data bits: " + std::to_string(databits);
        return false;
    }
    // 屏蔽其他标志位
    options.c_cflag &= ~CSIZE;
    switch (databits) {
        case 5:
            options.c_cflag |= CS5;
            break;
        case 6:
            options.c_cflag |= CS6;
            break;
        case 7:
            options.c_cflag |= CS7;
            break;
        case 8:
            options.c_cflag |= CS8;
            break;
    }

    // 校验位设置（字符直接匹配）
    switch (std::tolower(parity)) { // 统一转换为小写
        case 'n': // 无奇偶校验位。
            options.c_cflag &= ~PARENB;
            options.c_iflag &= ~INPCK;
            break;
        case 'o': // 设置为奇校验
            options.c_cflag |= (PARENB | PARODD);
            options.c_iflag |= INPCK;
            break;
        case 'e': // 设置为偶校验
            options.c_cflag |= PARENB;
            options.c_cflag &= ~PARODD;
            options.c_iflag |= INPCK;
            break;
        case 's': // 设置为空格
            options.c_cflag &= ~PARENB;
            break;
        default:
            _error_message = "Invalid parity: " + std::string(1, parity);
            return false;
    }

    // 停止位设置
    switch (stopbits) {
        case 1:
            options.c_cflag &= ~CSTOPB;
            break;
        case 2:
            options.c_cflag |= CSTOPB;
            break;
        default:
            _error_message = "Invalid stop bits: " + std::to_string(stopbits);
            return false;
    }

    // 输入输出模式优化
    options.c_oflag &= ~OPOST; // 原始输出
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 非规范模式
    // 传输特殊字符，否则特殊字符0x0d,0x11,0x13会被屏蔽或映射。
    options.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);

    // 设置等待时间和最小接收字符
    options.c_cc[VTIME] = 1; // 0.1秒超时
    options.c_cc[VMIN] = 1; // 至少读取1字符

    if (tcflush(_fd, TCIFLUSH) != 0) {
        _error_message = "tcflush failed: " + std::string(strerror(errno));
        return false;
    }

    // 激活配置 (将修改后的termios数据设置到串口中）
    if (tcsetattr(_fd, TCSANOW, &options) != 0) {
        _error_message = "tcsetattr failed: " + std::string(strerror(errno));
        return false;
    }

    return true;
}

bool UartProtocol::open() {
    if (_is_open) {
        return true;
    }
    _fd = ::open(_device_path.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (-1 == _fd) {
        _error_message = "can't open uart device: " + _device_path;
        return false;
    }
    // 恢复串口为阻塞状态
    if (fcntl(_fd, F_SETFL, 0) < 0) {
        _error_message = "fcntl failed";
        return false;
    }
    // 设置串口数据帧格式
    if (!set_param(_speed, _flow_ctrl, _databits, _stopbits, _parity)) {
        return false;
    }
    _is_open = true;
    return true;
}

[[nodiscard]] bool UartProtocol::is_open() const noexcept {
    return _is_open;
}

void UartProtocol::close() noexcept {
    if (!_is_open)
        return;

    // 检查 ::close 返回值并记录错误
    if (::close(_fd) == -1) {
        _error_message = std::strerror(errno);
    }
    _fd = -1;
    _is_open = false;
}

// 使用 std::byte 增强类型安全
[[nodiscard]] int UartProtocol::read(std::byte* buffer, std::size_t len) noexcept {
    const int ret = ::read(_fd, buffer, len);
    if (ret < 0) {
        _error_message = std::strerror(errno);
    }
    return ret;
}

[[nodiscard]] int UartProtocol::write(const std::byte* buffer, std::size_t len) noexcept {
    const int ret = ::write(_fd, buffer, len);
    if (ret < 0) {
        _error_message = std::strerror(errno);
    }
    return ret;
}
