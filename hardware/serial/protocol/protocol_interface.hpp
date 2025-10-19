#ifndef PROTOCOL_INTERFACE_HPP
#define PROTOCOL_INTERFACE_HPP

// C system headers

// C++ system headers
#include <cstddef>
#include <string>
#include <string_view>
#include <system_error>

// Third-party library headers

// Project headers

class ProtocolInterface
{
public:
    virtual ~ProtocolInterface() = default;

    /*基础操作*/
    [[nodiscard]] virtual bool open() = 0;
    virtual void close() noexcept = 0;
    [[nodiscard]] virtual bool is_open() const noexcept = 0;

    /*数据传输*/
    [[nodiscard]] virtual int read(std::byte *buffer, std::size_t len) = 0;
    [[nodiscard]] virtual int write(const std::byte *buffer, std::size_t len) = 0;

    /*错误信息*/
    [[nodiscard]] virtual std::string error_message() const = 0;
};

#endif // PROTOCOL_INTERFACE_HPP