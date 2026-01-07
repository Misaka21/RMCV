#include "plotter.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <mutex>

#include <fmt/format.h>

namespace plotter {

namespace {

// 全局单例，线程安全
class PlotterImpl {
public:
    static PlotterImpl& instance() {
        static PlotterImpl inst;
        return inst;
    }

    void init(const std::string& host, uint16_t port) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (socket_fd_ >= 0) {
            ::close(socket_fd_);
        }
        socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        dest_addr_.sin_family = AF_INET;
        dest_addr_.sin_port = htons(port);
        dest_addr_.sin_addr.s_addr = ::inet_addr(host.c_str());
    }

    void send(const std::string& json) {
        std::lock_guard<std::mutex> lock(mutex_);
        ensure_init();
        ::sendto(socket_fd_, json.c_str(), json.length(), 0,
                 reinterpret_cast<const sockaddr*>(&dest_addr_), sizeof(dest_addr_));
    }

private:
    PlotterImpl() = default;

    ~PlotterImpl() {
        if (socket_fd_ >= 0) {
            ::close(socket_fd_);
        }
    }

    // 懒初始化
    void ensure_init() {
        if (socket_fd_ < 0) {
            socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
            dest_addr_.sin_family = AF_INET;
            dest_addr_.sin_port = htons(9870);
            dest_addr_.sin_addr.s_addr = ::inet_addr("127.0.0.1");
        }
    }

    int socket_fd_ = -1;
    sockaddr_in dest_addr_{};
    std::mutex mutex_;
};

}  // namespace

void init(const std::string& host, uint16_t port) {
    PlotterImpl::instance().init(host, port);
}

void plot(const std::string& name, double value) {
    auto json = fmt::format("{{\"{}\":{}}}", name, value);
    PlotterImpl::instance().send(json);
}

void plot(const std::string& name, int value) {
    auto json = fmt::format("{{\"{}\":{}}}", name, value);
    PlotterImpl::instance().send(json);
}

void plot(const std::string& name, bool value) {
    auto json = fmt::format("{{\"{}\":{}}}", name, value ? 1 : 0);
    PlotterImpl::instance().send(json);
}

}  // namespace plotter
