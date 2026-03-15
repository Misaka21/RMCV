#ifndef PLUGIN_RERUN_RMCV_RERUN_HPP
#define PLUGIN_RERUN_RMCV_RERUN_HPP

#include <string>
#include <vector>

#include <Eigen/Core>

#include <opencv2/core.hpp>

namespace rr {

#ifdef ENABLE_RERUN

// 生命周期
void init();
void shutdown();
bool enabled();

// 标量 (替代 plotter::add + dashboard::set)
void scalar(const std::string& path, double value);
void scalar(const std::string& path, int value);
void scalar(const std::string& path, bool value);

// 时间
void set_time(double timestamp_s);
void set_time_us(int64_t timestamp_us);

// 图像 (内置跳帧 + 后台线程 resize+encode)
void image(const std::string& path, const cv::Mat& img, int skip_factor = 2);

// 3D
void points3d(const std::string& path,
              const std::vector<Eigen::Vector3d>& positions,
              uint8_t r = 255, uint8_t g = 255, uint8_t b = 255,
              float radius = 0.02f);

void arrows3d(const std::string& path,
              const std::vector<Eigen::Vector3d>& origins,
              const std::vector<Eigen::Vector3d>& vectors,
              uint8_t r = 255, uint8_t g = 255, uint8_t b = 255);

// 文本
void text(const std::string& path, const std::string& body);

#else  // !ENABLE_RERUN

// 空实现 - 编译期消除所有开销
inline void init() {}
inline void shutdown() {}
inline bool enabled() { return false; }

inline void scalar([[maybe_unused]] const std::string& path, [[maybe_unused]] double value) {}
inline void scalar([[maybe_unused]] const std::string& path, [[maybe_unused]] int value) {}
inline void scalar([[maybe_unused]] const std::string& path, [[maybe_unused]] bool value) {}

inline void set_time([[maybe_unused]] double timestamp_s) {}
inline void set_time_us([[maybe_unused]] int64_t timestamp_us) {}

inline void image([[maybe_unused]] const std::string& path,
                  [[maybe_unused]] const cv::Mat& img,
                  [[maybe_unused]] int skip_factor = 2) {}

inline void points3d([[maybe_unused]] const std::string& path,
                     [[maybe_unused]] const std::vector<Eigen::Vector3d>& positions,
                     [[maybe_unused]] uint8_t r = 255,
                     [[maybe_unused]] uint8_t g = 255,
                     [[maybe_unused]] uint8_t b = 255,
                     [[maybe_unused]] float radius = 0.02f) {}

inline void arrows3d([[maybe_unused]] const std::string& path,
                     [[maybe_unused]] const std::vector<Eigen::Vector3d>& origins,
                     [[maybe_unused]] const std::vector<Eigen::Vector3d>& vectors,
                     [[maybe_unused]] uint8_t r = 255,
                     [[maybe_unused]] uint8_t g = 255,
                     [[maybe_unused]] uint8_t b = 255) {}

inline void text([[maybe_unused]] const std::string& path,
                 [[maybe_unused]] const std::string& body) {}

#endif  // ENABLE_RERUN

}  // namespace rr

#endif  // PLUGIN_RERUN_RMCV_RERUN_HPP
