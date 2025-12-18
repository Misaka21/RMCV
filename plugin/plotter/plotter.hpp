#ifndef PLUGIN_PLOTTER_PLOTTER_HPP
#define PLUGIN_PLOTTER_PLOTTER_HPP

#include <string>

namespace plotter {

/**
 * 初始化 PlotJuggler 连接 (可选，不调用则使用默认配置)
 * @param host PlotJuggler 地址，默认 127.0.0.1
 * @param port PlotJuggler 端口，默认 9870
 */
void init(const std::string& host = "127.0.0.1", uint16_t port = 9870);

/**
 * 发送单个数据点到 PlotJuggler
 * @param name 数据名称，如 "/target.yaw" 或 "gimbal/pitch"
 * @param value 数据值
 *
 * 使用示例:
 *   plotter::plot("/target.yaw", yaw);
 *   plotter::plot("/gimbal.pitch", pitch);
 *   plotter::plot("/distance", 5.2);
 */
void plot(const std::string& name, double value);

/**
 * 发送 int 类型数据
 */
void plot(const std::string& name, int value);

/**
 * 发送 bool 类型数据 (转为 0/1)
 */
void plot(const std::string& name, bool value);

}  // namespace plotter

#endif  // PLUGIN_PLOTTER_PLOTTER_HPP
