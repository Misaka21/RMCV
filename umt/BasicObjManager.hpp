//
// Created by david on 24-11-28.
//

#ifndef UMT_BASICOBJMANAGER_HPP
#define UMT_BASICOBJMANAGER_HPP

#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <type_traits>
#include <unordered_map>

namespace umt {

/**
 * @brief 基本类型和结构体的包装类
 * @tparam T 要包装的类型
 */
template<typename T>
class BasicTypeWrapper {
public:
    template<typename... Args>
    explicit BasicTypeWrapper(Args&&... args): value(std::forward<Args>(args)...) {}

    T& get() {
        return value;
    }

    const T& get() const {
        return value;
    }

    void set(const T& new_value) {
        value = new_value;
    }

    void set(T&& new_value) {
        value = std::move(new_value);
    }

private:
    T value;
};

/**
 * @brief 基本类型的命名共享对象管理器
 *
 * @details 这是一个线程安全的对象管理器，用于管理具有名称的共享对象。
 * 支持基本类型（如int、double等）和结构体。
 * 使用强引用确保对象在显式删除前持续存在。
 * 所有操作都是线程安全的。
 *
 * @tparam T 要管理的类型
 */
template<typename T>
class BasicObjManager {
public:
    using WrapperType = BasicTypeWrapper<T>;
    using sptr = std::shared_ptr<WrapperType>;
    using wptr = std::weak_ptr<WrapperType>;

    /**
		 * @brief 创建一个命名共享对象
		 *
		 * @details 如果指定名称的对象已存在，则返回nullptr。
		 * 创建的对象会被管理器持有强引用，直到显式删除。
		 *
		 * @tparam Args 构造参数类型包
		 * @param name 对象名称
		 * @param args 构造参数
		 * @return sptr 创建的共享对象指针，如果名称已存在则返回nullptr
		 */
    template<typename... Args>
    static sptr create(const std::string& name, Args&&... args) {
        std::unique_lock lock(_mtx);
        if (_map.find(name) != _map.end()) return nullptr;

        sptr p_obj = std::make_shared<WrapperType>(std::forward<Args>(args)...);
        _map.emplace(name, p_obj);
        _strong_refs[name] = p_obj;
        _names.insert(name);
        return p_obj;
    }

    /**
		 * @brief 查找一个命名共享对象
		 *
		 * @param name 对象名称
		 * @return sptr 找到的共享对象指针，如果不存在则返回nullptr
		 */
    static sptr find(const std::string& name) {
        std::unique_lock lock(_mtx);
        auto iter = _strong_refs.find(name);
        if (iter == _strong_refs.end()) return nullptr;
        return iter->second;
    }

    /**
		 * @brief 查找一个命名共享对象，如果不存在则创建
		 *
		 * @tparam Args 构造参数类型包
		 * @param name 对象名称
		 * @param args 构造参数
		 * @return sptr 查找到的或新创建的共享对象指针
		 */
    template<typename... Args>
    static sptr find_or_create(const std::string& name, Args&&... args) {
        std::unique_lock lock(_mtx);
        auto iter = _strong_refs.find(name);
        if (iter != _strong_refs.end()) return iter->second;

        sptr p_obj = std::make_shared<WrapperType>(std::forward<Args>(args)...);
        _map[name] = p_obj;
        _strong_refs[name] = p_obj;
        _names.insert(name);
        return p_obj;
    }

    /**
		 * @brief 删除指定名称的对象
		 *
		 * @details 这将移除管理器对对象的强引用。
		 * 如果没有其他引用持有该对象，它将被销毁。
		 *
		 * @param name 要删除的对象名称
		 */
    static void remove(const std::string& name) {
        std::unique_lock lock(_mtx);
        _map.erase(name);
        _strong_refs.erase(name);
        _names.erase(name);
    }

    /**
		 * @brief 获取所有管理的对象名称
		 *
		 * @return std::set<std::string> 当前管理的所有对象名称集合
		 */
    static std::set<std::string> names() {
        std::unique_lock lock(_mtx);
        return _names;
    }

private:
    static std::mutex _mtx; ///< 全局互斥锁
		static std::unordered_map<std::string, wptr> _map;             ///< 弱引用映射
		static std::unordered_map<std::string, sptr> _strong_refs;     ///< 强引用映射
		static std::set<std::string> _names;                           ///< 名称集合
	};

// 静态成员初始化
	template<typename T> std::mutex BasicObjManager<T>::_mtx;
	template<typename T> std::unordered_map<std::string, typename BasicObjManager<T>::wptr> BasicObjManager<T>::_map;
	template<typename T> std::unordered_map<std::string, typename BasicObjManager<T>::sptr> BasicObjManager<T>::_strong_refs;
	template<typename T> std::set<std::string> BasicObjManager<T>::_names;

} // namespace umt


#endif //UMT_BASICOBJMANAGER_HPP