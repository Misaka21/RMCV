//
// Dashboard - 动态数据注册系统
// 用法: dashboard::set("/detector.fps", fps);
//       dashboard::set("/target.yaw", yaw);
//

#ifndef DASHBOARD_HPP
#define DASHBOARD_HPP

#include <string>
#include <unordered_map>
#include <variant>
#include <shared_mutex>
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

    // 设置值
    template<typename T>
    void set(const std::string& key, const T& value) {
        std::unique_lock lock(mutex_);
        data_[key] = value;
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

    // 清空
    void clear() {
        std::unique_lock lock(mutex_);
        data_.clear();
    }

private:
    Registry() = default;
    mutable std::shared_mutex mutex_;
    std::unordered_map<std::string, Value> data_;
};

// 便捷函数
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

}  // namespace dashboard

#endif  // DASHBOARD_HPP
