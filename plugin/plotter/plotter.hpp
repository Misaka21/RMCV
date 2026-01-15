#ifndef PLUGIN_PLOTTER_PLOTTER_HPP
#define PLUGIN_PLOTTER_PLOTTER_HPP

#include <string>

namespace plotter {

/**
 * 初始化 Plotter (从 debugger.toml 读取配置)
 */
void init();

/**
 * 设置全局开关
 */
void set_enabled(bool enabled);

/**
 * 单条发送
 */
void plot(const std::string& name, double value);
void plot(const std::string& name, int value);
void plot(const std::string& name, bool value);

/**
 * 批量发送 - 每个线程独立缓冲区
 *
 * 用法:
 *   plotter::begin();  // 可选，清空缓冲区
 *   plotter::add("/target/yaw", 1.0);
 *   plotter::add("/target/pitch", 2.0);
 *   plotter::end();    // 发送
 */
void begin();  // 清空缓冲区 (可选)
void add(const std::string& name, double value);
void add(const std::string& name, int value);
void add(const std::string& name, bool value);
void end();    // 发送并清空

}  // namespace plotter

#endif  // PLUGIN_PLOTTER_PLOTTER_HPP
