//
// Created by nuc11 on 2025/10/2.
//

#ifndef RMCV2026_STATIC_CONFIG_HPP
#define RMCV2026_STATIC_CONFIG_HPP

// C system headers

// C++ system headers
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

// Third-party library headers
#include <toml++/toml.hpp>

// Project headers
#include "plugin/debug/logger.hpp"

// 定义 Param variant 类型
using Param = std::variant<bool, int64_t, double, std::string, std::vector<int64_t> >;

namespace static_param {
    // 内部辅助函数，将 toml::node 转换为我们的 Param variant
    // 与之前相同，这是核心转换逻辑
    inline Param get_value(const toml::node &node) {
        if (node.is_boolean()) {
            return (*node.as_boolean()).get();
        }
        if (node.is_integer()) {
            return (*node.as_integer()).get();
        }
        if (node.is_floating_point()) {
            return (*node.as_floating_point()).get();
        }
        if (node.is_string()) {
            return (*node.as_string()).get();
        }
        if (node.is_array() && (node.as_array()->is_homogeneous<int64_t>() || node.as_array()->empty())) {
            std::vector<int64_t> vec_int;
            for (const auto &ele: *node.as_array()) {
                vec_int.push_back((*ele.as_integer()).get());
            }
            return vec_int;
        }
        // 如果遇到不支持的类型（例如，一个浮点数组或一个嵌套表）
        throw std::runtime_error("Unsupported TOML value type encountered.");
    }


    /**
     * @brief 从配置目录加载并解析 TOML 文件（自动添加 CONFIG_DIR 前缀）。
     *
     * @param filename TOML 文件名（不需要路径前缀）。
     * @return toml::table 解析后的 table 对象。
     * @throws std::runtime_error 如果解析失败，抛出异常。
     */
    inline toml::table parse_file(const std::string &filename) {
        try {
            return toml::parse_file(CONFIG_DIR"/" + filename);
        } catch (const toml::parse_error &err) {
            debug::print("error", "static_param", "Failed to parse config file '{}': {}", filename, err.what());
            throw;
        }
    }

    /**
    * @brief 从解析好的 TOML table 中获取一个指定类型的参数。
    *
    * @tparam T 你期望获取的类型 (e.g., int64_t, double, std::string)。
    * @param data 由 parse_file 返回的 toml::table 对象。
    * @param table_name 参数所在的表名 (e.g., "Camera")。
    * @param key_name 参数的键名 (e.g., "id")。
    * @return T 获取到的参数值。如果找不到或类型不匹配，返回一个默认构造的 T() 并打印错误信息。
    */
    template<typename T>
    T get_param(const toml::table &data, const std::string &table_name, const std::string &key_name) {
        std::string path = table_name + "." + key_name;
        const toml::node *node = data.at_path(path).node();

        if (!node) {
            debug::print("error",
                         "static_param",
                         "Parameter \"{}\" not found. Returning default value.",
                         path);
            return T{};
        }

        try {
            Param param_variant = get_value(*node);
            if (auto val = std::get_if<T>(&param_variant)) {
                return *val;
            } else {
                // 为了更好地报告错误，我们可以访问原始值
                debug::print("error",
                             "static_param",
                             "Parameter \"{}\" found but type mismatch.Expected type that holds T,\
                                but got incompatible type. Returning default value.",
                             path);
                return T{};
            }
        } catch (const std::runtime_error &e) {
            debug::print("error",
                         "static_param",
                         "Failed to convert parameter \"{}\" :{}. Returning default value.",
                         path, e.what());
            return T{};
        }
    }

    /**
     * @brief 从解析好的 TOML table 中获取一个子表，将其转换为键值对列表。
     * 支持嵌套表路径，如 "Camera.config"。
     *
     * @param data 由 parse_file 返回的 toml::table 对象。
     * @param table_path 要获取的子表路径 (e.g., "Camera.config" 或 "config")。
     * @return std::vector<std::pair<std::string, Param>> 包含子表中所有键值对的 vector。
     *         如果找不到表，返回一个空的 vector。
     */
    inline std::vector<std::pair<std::string, Param> > get_param_table(const toml::table &data,
                                                                       const std::string &table_path) {
        // 查找嵌套表路径
        const toml::node *node = data.at_path(table_path).node();
        const toml::table *sub_table = node ? node->as_table() : nullptr;

        if (!sub_table) {
            debug::print("error",
                         "static_param",
                         "Table \"{}\" not found. Returning empty table.",
                         table_path);
            return {};
        }

        std::vector<std::pair<std::string, Param> > result;
        for (const auto &[key, node]: *sub_table) {
            try {
                Param value = get_value(node);
                result.emplace_back(std::string(key), value);
            } catch (const std::runtime_error &e) {
                debug::print("error",
                             "static_param",
                             "Skipping key \"{}\" in table \"{}\" due to error:{}",
                             key, table_path, e.what());
            }
        }
        return result;
    }
}

#endif //RMCV2026_STATIC_CONFIG_HPP
