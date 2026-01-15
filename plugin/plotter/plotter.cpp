#include "plotter.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <mutex>
#include <string>

#include <fmt/format.h>

#include "plugin/param/static_config.hpp"

namespace plotter {

namespace {

// 全局开关
std::atomic<bool> g_enabled{false};

// 每个线程独立的缓冲区
thread_local std::string t_buffer;

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
        if (socket_fd_ >= 0) ::close(socket_fd_);
    }

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

void init() {
    auto cfg = static_param::parse_file("debugger.toml");
    bool enabled = static_param::get_param<bool>(cfg, "Plotter", "enable_plotter");
    std::string host = static_param::get_param<std::string>(cfg, "Plotter", "host");
    int64_t port = static_param::get_param<int64_t>(cfg, "Plotter", "port");

    g_enabled.store(enabled, std::memory_order_relaxed);
    if (enabled) {
        PlotterImpl::instance().init(host, static_cast<uint16_t>(port));
    }
}

void set_enabled(bool enabled) {
    g_enabled.store(enabled, std::memory_order_relaxed);
}

// 单条发送
void plot(const std::string& name, double value) {
    if (!g_enabled.load(std::memory_order_relaxed)) return;
    PlotterImpl::instance().send(fmt::format("{{\"{}\":{}}}", name, value));
}

void plot(const std::string& name, int value) {
    if (!g_enabled.load(std::memory_order_relaxed)) return;
    PlotterImpl::instance().send(fmt::format("{{\"{}\":{}}}", name, value));
}

void plot(const std::string& name, bool value) {
    if (!g_enabled.load(std::memory_order_relaxed)) return;
    PlotterImpl::instance().send(fmt::format("{{\"{}\":{}}}", name, value ? 1 : 0));
}

// 批量发送
void begin() {
    t_buffer.clear();
}

void add(const std::string& name, double value) {
    if (!g_enabled.load(std::memory_order_relaxed)) return;
    if (!t_buffer.empty()) t_buffer += ',';
    fmt::format_to(std::back_inserter(t_buffer), "\"{}\":{}", name, value);
}

void add(const std::string& name, int value) {
    if (!g_enabled.load(std::memory_order_relaxed)) return;
    if (!t_buffer.empty()) t_buffer += ',';
    fmt::format_to(std::back_inserter(t_buffer), "\"{}\":{}", name, value);
}

void add(const std::string& name, bool value) {
    if (!g_enabled.load(std::memory_order_relaxed)) return;
    if (!t_buffer.empty()) t_buffer += ',';
    fmt::format_to(std::back_inserter(t_buffer), "\"{}\":{}", name, value ? 1 : 0);
}

void end() {
    if (!g_enabled.load(std::memory_order_relaxed) || t_buffer.empty()) return;
    PlotterImpl::instance().send("{" + t_buffer + "}");
    t_buffer.clear();
}

}  // namespace plotter
