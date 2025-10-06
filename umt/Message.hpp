#ifndef _UMT_MESSAGE_HPP_
#define _UMT_MESSAGE_HPP_

#include <condition_variable>
#include <list>
#include <mutex>
#include <queue>
#include <chrono>
#include <atomic>
#include <algorithm>
#include <cmath>

#include <pybind11/embed.h>
#include <pybind11/pybind11.h>

#include "ObjManager.hpp"

namespace umt {
    /**
 * @brief 消息异常类型
 */
    class MessageError : public std::runtime_error {
    protected:
        using std::runtime_error::runtime_error;
    };

    /**
 * @brief 消息异常类型，当前消息上无publisher
 */
    class MessageError_Stopped : public MessageError {
    public:
        MessageError_Stopped() : MessageError("no publisher on this message!") {
        }
    };

    /**
 * @brief 消息异常类型，消息读取超时
 */
    class MessageError_Timeout : public MessageError {
    public:
        MessageError_Timeout() : MessageError("message read timeout!") {
        }
    };

    /**
 * @brief 消息异常类型，空消息（未初始化或使用过std::move）
 */
    class MessageError_Empty : public MessageError {
    public:
        MessageError_Empty()
            : MessageError("empty message. maybe uninitailized or moved!") {
        }
    };

    template<class T>
    class Publisher;

    template<class T>
    class Subscriber;

    namespace utils {
        template<class T>
        class MessagePipe {
            friend class Publisher<T>;

            friend class Subscriber<T>;

        public:
            using MsgType = T;

        private:
            std::mutex pubs_mtx;
            std::list<Publisher<T> *> pubs;
            std::mutex subs_mtx;
            std::list<Subscriber<T> *> subs;
        };
    } // namespace utils

    /**
 * @brief 消息订阅器类型
 * @details
 * 使用队列存储收到的消息，可以设置队列最大长度，当超出最大队列长度时，新消息会覆盖最老的消息
 * @tparam T 消息对象类型
 */
    template<class T>
    class Subscriber {
        friend Publisher<T>;

    private:
        using MsgManager = ObjManager<utils::MessagePipe<T> >;

    public:
        using MsgType = T;

        Subscriber() = default;

        /**
   * @details 构造函数
   * @param msg_name 消息名称
   * @param max_fifo_size 最大消息长度
   */
        explicit Subscriber(const std::string &msg_name, size_t size = 1)
            : fifo_size(size) {
            bind(msg_name);
        }

        /// 拷贝构造函数
        Subscriber(const Subscriber &other)
            : fifo_size(other.fifo_size), fifo(other.fifo), p_msg(other.p_msg) {
            std::unique_lock subs_lock(p_msg->subs_mtx);
            p_msg->subs.emplace_front(this);
        }

        /// 移动构造函数
        Subscriber(Subscriber &&other) noexcept
            : fifo_size(other.fifo_size),
              fifo(std::move(other.fifo)),
              p_msg(other.p_msg) {
            other.reset();
            std::unique_lock subs_lock(p_msg->subs_mtx);
            p_msg->subs.emplace_front(this);
        }

        /// 析构函数
        ~Subscriber() { reset(); }

        /// 判断当前订阅器是否绑定到某个消息
        explicit operator bool() { return p_msg != nullptr; }

        /// 重置订阅器
        void reset() {
            if (!fifo.empty())
                fifo = std::queue<T>();
            if (!p_msg)
                return;
            std::unique_lock subs_lock(p_msg->subs_mtx);
            p_msg->subs.remove(this);
            p_msg.reset();
        }

        /**
   * @brief 绑定当前订阅器到某个名称的消息
   * @param msg_name 消息名称
   */
        void bind(const std::string &msg_name) {
            reset();
            p_msg = MsgManager::find_or_create(msg_name);
            std::unique_lock subs_lock(p_msg->subs_mtx);
            p_msg->subs.emplace_front(this);
        }

        /**
   * @brief 清空接收缓冲区
   */
        void clear() {
            std::unique_lock lock(mtx);
            fifo = std::queue<T>();
        }

        /**
   * @brief 设置队列长度，size==0则不限制最大长度
   * @param size 最大队列长度
   */
        void set_fifo_size(size_t size) { fifo_size = size; }

        /**
   * @brief 读取当前最大队列长度
   * @return 当前最大队列长度
   */
        size_t get_fifo_size() { return fifo_size; }

        /**
   * @brief 尝试获取一条消息
   * @details 如果当前消息上没有发布器，则会抛出一条异常
   * @return 读取到的消息
   */
        T pop() {
            if (!p_msg)
                throw MessageError_Empty();
            std::unique_lock lock(mtx);
            cv.wait(lock, [this]() { return p_msg->pubs.empty() || !fifo.empty(); });
            if (p_msg->pubs.empty())
                throw MessageError_Stopped();
            T tmp = std::move(fifo.front());
            fifo.pop();
            return tmp;
        }

        /**
   * @brief 尝试获取一条消息，有超时时间
   * @details
   * 如果当前消息上没有发布器，则会抛出一条异常；如果超时，也会抛出一条异常
   * @param ms 超时时间，单位毫秒
   * @return 读取到的消息
   */
        T pop_for(size_t ms) {
            if (!p_msg)
                throw MessageError_Empty();
            using namespace std::chrono;
            std::unique_lock lock(mtx);
            if (!cv.wait_for(lock, milliseconds(ms), [this]() {
                return p_msg->pubs.empty() || !fifo.empty();
            })) {
                throw MessageError_Timeout();
            }
            if (p_msg->pubs.empty())
                throw MessageError_Stopped();
            T tmp = std::move(fifo.front());
            fifo.pop();
            return tmp;
        }

        /**
   * @brief 尝试获取一条消息，直到某个时间点超时
   * @details
   * 如果当前消息上没有发布器，则会抛出一条异常；如果超时，也会抛出一条异常
   * @param pt 超时时间点，为std::chrono::time_point类型
   * @return 读取到的消息
   */
        template<class P>
        T pop_until(P pt) {
            if (!p_msg)
                throw MessageError_Empty();
            std::unique_lock lock(mtx);
            if (!cv.wait_until(lock, pt, [this]() {
                return p_msg->pubs.empty() || !fifo.empty();
            })) {
                throw MessageError_Timeout();
            }
            if (p_msg->pubs.empty())
                throw MessageError_Stopped();
            T tmp = std::move(fifo.front());
            fifo.pop();
            return tmp;
        }

    private:
        void write_obj(const T &obj) {
            std::unique_lock lock(mtx);
            if (fifo_size > 0 && fifo.size() >= fifo_size) {
                fifo.pop();
            }
            fifo.push(obj);
        }

        void notify() const {
            cv.notify_one();
            // 更新性能统计
            update_performance_stats();
        }

        /**
         * @brief 获取消息发布性能统计信息
         * @details 计算开销极小，使用原子操作和环形缓冲区
         * @return 包含频率、延迟等统计信息的结构体
         */
        struct PerformanceStats {
            double avg_frequency_hz;    // 1秒内平均发布频率 (Hz)
            double max_latency_ms;     // 最大延迟 (毫秒)
            double p1_latency_ms;      // 1%低延迟 (毫秒)
            uint64_t total_messages;    // 总消息数
            double window_duration_s;   // 统计窗口时长 (秒)
        };

        PerformanceStats get_performance_stats() const {
            PerformanceStats stats;
            const auto now = std::chrono::steady_clock::now();
            const auto window_start = now - std::chrono::seconds(1);

            // 计算最近1秒内的消息数量
            uint64_t recent_count = 0;
            {
                std::lock_guard<std::mutex> lock(perf_mutex);
                size_t start_idx = latency_buffer_idx;

                // 找到1秒时间窗口内的第一条消息
                while (start_idx > 0 && latency_buffer[start_idx].timestamp > window_start) {
                    start_idx--;
                }

                // 统计最近1秒内的消息数量
                for (size_t i = start_idx; i <= latency_buffer_idx; i++) {
                    if (latency_buffer[i].timestamp >= window_start) {
                        recent_count++;
                    }
                }

                stats.total_messages = total_messages.load();
                stats.window_duration_s = 1.0;
                stats.avg_frequency_hz = static_cast<double>(recent_count);
            }

            // 计算延迟统计
            if (recent_count > 0) {
                std::vector<double> recent_latencies;
                recent_latencies.reserve(recent_count);

                {
                    std::lock_guard<std::mutex> lock(perf_mutex);
                    size_t start_idx = latency_buffer_idx;

                    // 找到1秒时间窗口内的第一条消息
                    while (start_idx > 0 && latency_buffer[start_idx].timestamp > window_start) {
                        start_idx--;
                    }

                    // 收集最近1秒内的延迟数据
                    for (size_t i = start_idx; i <= latency_buffer_idx; i++) {
                        if (latency_buffer[i].timestamp >= window_start) {
                            recent_latencies.push_back(latency_buffer[i].latency_ms);
                        }
                    }
                }

                if (!recent_latencies.empty()) {
                    // 计算最大延迟
                    stats.max_latency_ms = *std::max_element(recent_latencies.begin(), recent_latencies.end());

                    // 计算1%低延迟 (P1)
                    std::sort(recent_latencies.begin(), recent_latencies.end());
                    size_t p1_idx = static_cast<size_t>(recent_latencies.size() * 0.01);
                    if (p1_idx >= recent_latencies.size()) p1_idx = recent_latencies.size() - 1;
                    stats.p1_latency_ms = recent_latencies[p1_idx];
                }
            }

            return stats;
        }

        /**
         * @brief 打印性能统计信息
         */
        void print_performance_stats() const {
            auto stats = get_performance_stats();
            printf("Message Performance Stats:\n");
            printf("  Frequency: %.2f Hz\n", stats.avg_frequency_hz);
            printf("  Max Latency: %.3f ms\n", stats.max_latency_ms);
            printf("  P1 Latency: %.3f ms\n", stats.p1_latency_ms);
            printf("  Total Messages: %lu\n", stats.total_messages);
        }

    private:
        mutable std::mutex mtx;
        mutable std::condition_variable cv;
        size_t fifo_size{};
        std::queue<T> fifo;
        typename MsgManager::sptr p_msg;

        // 性能统计相关成员变量
        mutable std::mutex perf_mutex;
        static constexpr size_t LATENCY_BUFFER_SIZE = 1024;  // 环形缓冲区大小

        struct LatencyRecord {
            std::chrono::steady_clock::time_point timestamp;
            double latency_ms;
        };

        mutable std::array<LatencyRecord, LATENCY_BUFFER_SIZE> latency_buffer;
        mutable size_t latency_buffer_idx = 0;
        mutable std::atomic<uint64_t> total_messages{0};
        mutable std::chrono::steady_clock::time_point last_message_time;

        /**
         * @brief 更新性能统计
         * @details 开销极小：2次时间戳获取 + 1次原子操作 + 数组写入
         */
        void update_performance_stats() const {
            const auto now = std::chrono::steady_clock::now();
            double latency_ms = 0.0;

            // 计算与上一条消息的延迟
            if (total_messages.load() > 0) {
                auto duration = std::chrono::duration<double, std::milli>(now - last_message_time);
                latency_ms = duration.count();
            }

            // 更新统计信息
            total_messages.fetch_add(1, std::memory_order_relaxed);
            last_message_time = now;

            // 写入环形缓冲区
            {
                std::lock_guard<std::mutex> lock(perf_mutex);
                latency_buffer[latency_buffer_idx] = {now, latency_ms};
                latency_buffer_idx = (latency_buffer_idx + 1) % LATENCY_BUFFER_SIZE;
            }
        }
    };

    template<class T>
    class Publisher {
    private:
        using MsgManager = ObjManager<utils::MessagePipe<T> >;

    public:
        using MsgType = T;

        Publisher() = default;

        /**
   * @brief 发布器的构造函数
   * @param msg_name 消息名称
   */
        explicit Publisher(const std::string &msg_name) { bind(msg_name); }

        /// 拷贝构造函数
        Publisher(const Publisher &other) : p_msg(other.p_msg) {
            std::unique_lock pubs_lock(p_msg->pubs_mtx);
            p_msg->pubs.emplace_front(this);
        }

        /// 移动构造函数
        Publisher(Publisher &&other) noexcept : p_msg(other.p_msg) {
            other.reset();
            std::unique_lock pubs_lock(p_msg->pubs_mtx);
            p_msg->pubs.emplace_front(this);
        }

        /// 析构函数
        ~Publisher() { reset(); }

        /// 判断当前发布器是否绑定到某个消息
        explicit operator bool() { return p_msg != nullptr; }

        /// 重置发布器
        void reset() {
            if (!p_msg)
                return;
            std::unique_lock pubs_lock(p_msg->pubs_mtx);
            p_msg->pubs.remove(this);
            if (p_msg->pubs.empty()) {
                std::unique_lock subs_lock(p_msg->subs_mtx);
                for (const auto &sub: p_msg->subs) {
                    sub->notify();
                }
            }
            p_msg.reset();
        }

        /**
   * @brief 绑定当前发布器到某个名称的消息
   * @param msg_name 消息名称
   */
        void bind(const std::string &msg_name) {
            reset();
            p_msg = MsgManager::find_or_create(msg_name);
            std::unique_lock pubs_lock(p_msg->pubs_mtx);
            p_msg->pubs.emplace_front(this);
        }

        /**
   * @brief 发布一条消息
   * @param obj 待发布的消息消息
   */
        void push(const T &obj) {
            if (!p_msg)
                throw MessageError_Empty();
            std::unique_lock subs_lock(p_msg->subs_mtx);
            for (auto &sub: p_msg->subs) {
                sub->write_obj(obj);
                sub->notify();
            }
        }

    private:
        typename MsgManager::sptr p_msg;
    };
} // namespace umt

#define UMT_EXPORT_MESSAGE_ALIAS_WITHOUT_TYPE_EXPORT(name, type, var) \
  PYBIND11_EMBEDDED_MODULE(Message_##name, m) {                       \
    using namespace umt;                                              \
    using namespace umt::utils;                                       \
    namespace py = pybind11;                                          \
    m.def("names", &ObjManager<MessagePipe<type>>::names);            \
    py::class_<Publisher<type>>(m, "Publisher")                       \
        .def(py::init<>())                                            \
        .def(py::init<std::string>(), py::arg("msg_name"))            \
        .def("reset", &Publisher<type>::reset)                        \
        .def("bind", &Publisher<type>::bind)                          \
        .def("push", &Publisher<type>::push);                         \
    py::class_<Subscriber<type>>(m, "Subscriber")                     \
        .def(py::init<>())                                            \
        .def(py::init<std::string, size_t>(), py::arg("msg_name"),    \
             py::arg("fifo_size") = 0)                                \
        .def("reset", &Subscriber<type>::reset)                       \
        .def("bind", &Subscriber<type>::bind)                         \
        .def("clear", &Subscriber<type>::clear)                       \
        .def("pop", &Subscriber<type>::pop)                           \
        .def("pop_for", &Subscriber<type>::pop_for);                  \
  }

#define UMT_EXPORT_MESSAGE_ALIAS(name, type, var)                  \
  void __umt_init_message_##name(pybind11::class_<type>&& var);    \
  PYBIND11_EMBEDDED_MODULE(Message_##name, m) {                    \
    using namespace umt;                                           \
    using namespace umt::utils;                                    \
    namespace py = pybind11;                                       \
    m.def("names", &ObjManager<MessagePipe<type>>::names);         \
    py::class_<Publisher<type>>(m, "Publisher")                    \
        .def(py::init<>())                                         \
        .def(py::init<std::string>(), py::arg("msg_name"))         \
        .def("reset", &Publisher<type>::reset)                     \
        .def("bind", &Publisher<type>::bind)                       \
        .def("push", &Publisher<type>::push);                      \
    py::class_<Subscriber<type>>(m, "Subscriber")                  \
        .def(py::init<>())                                         \
        .def(py::init<std::string, size_t>(), py::arg("msg_name"), \
             py::arg("fifo_size") = 0)                             \
        .def("reset", &Subscriber<type>::reset)                    \
        .def("bind", &Subscriber<type>::bind)                      \
        .def("clear", &Subscriber<type>::clear)                    \
        .def("pop", &Subscriber<type>::pop)                        \
        .def("pop_for", &Subscriber<type>::pop_for);               \
    try {                                                          \
      __umt_init_message_##name(                                   \
          py::class_<type, std::shared_ptr<type>>(m, #name));      \
    } catch (...) {                                                \
    }                                                              \
  }                                                                \
  void __umt_init_message_##name(pybind11::class_<type>&& var)

#endif /* _UMT_MESSAGE_HPP_ */