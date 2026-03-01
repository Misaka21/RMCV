//
// Dashboard - 动态数据注册系统
// 用法: dashboard::set("detector.fps", fps);
//       dashboard::set("target.yaw", yaw);
//

#ifndef DASHBOARD_HPP
#define DASHBOARD_HPP

#include <string>

#ifdef ENABLE_WEBVIEW

#include <atomic>
#include <mutex>
#include <shared_mutex>
#include <unordered_map>
#include <variant>
#include <vector>

namespace dashboard {

// 支持的数据类型
using Value = std::variant<int, float, double, bool, std::string>;

// 全局数据存储 (线程安全)
class Registry {
public:
    static Registry& instance() {
        static Registry inst;
        return inst;
    }

    // 启用遥测 (仅在 --web 模式下调用)
    void enable() { enabled_.store(true, std::memory_order_relaxed); }
    bool enabled() const { return enabled_.load(std::memory_order_relaxed); }

    // 设置值 (非 web 模式下几乎零开销)
    template<typename T>
    void set(const std::string& key, const T& value) {
        if (!enabled_.load(std::memory_order_relaxed)) return;
        std::unique_lock lock(mutex_);
        data_[key] = value;
        version_.fetch_add(1, std::memory_order_relaxed);
    }

    // 获取值
    Value get(const std::string& key) const {
        std::shared_lock lock(mutex_);
        auto it = data_.find(key);
        if (it != data_.end()) {
            return it->second;
        }
        return 0;  // 默认返回0
    }

    // 获取所有键
    std::vector<std::string> keys() const {
        std::shared_lock lock(mutex_);
        std::vector<std::string> result;
        result.reserve(data_.size());
        for (const auto& [k, v] : data_) {
            result.push_back(k);
        }
        return result;
    }

    // 获取所有数据 (用于Python批量读取)
    std::unordered_map<std::string, Value> all() const {
        std::shared_lock lock(mutex_);
        return data_;
    }

    // 版本号 (每次 set 递增)
    uint64_t version() const {
        return version_.load(std::memory_order_relaxed);
    }

    // 清空
    void clear() {
        std::unique_lock lock(mutex_);
        data_.clear();
        version_.fetch_add(1, std::memory_order_relaxed);
    }

private:
    Registry() = default;
    std::atomic<bool> enabled_{false};
    mutable std::shared_mutex mutex_;
    std::unordered_map<std::string, Value> data_;
    std::atomic<uint64_t> version_{0};
};

// 便捷函数
inline void enable() {
    Registry::instance().enable();
}

template<typename T>
inline void set(const std::string& key, const T& value) {
    Registry::instance().set(key, value);
}

inline Value get(const std::string& key) {
    return Registry::instance().get(key);
}

inline std::vector<std::string> keys() {
    return Registry::instance().keys();
}

inline std::unordered_map<std::string, Value> all() {
    return Registry::instance().all();
}

inline uint64_t version() {
    return Registry::instance().version();
}

}  // namespace dashboard

#else  // !ENABLE_WEBVIEW

// 空实现 - 不启用 ENABLE_WEBVIEW 时 dashboard::set() 什么都不做
namespace dashboard {

template<typename T>
inline void set([[maybe_unused]] const std::string& key, [[maybe_unused]] const T& value) {
    // no-op
}

}  // namespace dashboard

#endif  // ENABLE_WEBVIEW

#endif  // DASHBOARD_HPP
