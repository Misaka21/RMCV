//
// 验证海康相机 nHostTimeStamp 和 steady_clock 的关系
//
// 目的：
//   1. 确定 nHostTimeStamp 的单位（ns? us? ms?）
//   2. 验证它和 steady_clock 是否同源
//   3. 如果同源，可以直接用于时间同步标定
//

#include <chrono>
#include <thread>
#include <vector>
#include <cmath>

#include <fmt/format.h>
#include <fmt/color.h>

#include "hardware/hik_cam/hik_camera.hpp"
#include "plugin/param/static_config.hpp"

using namespace std::chrono;
using SteadyClock = std::chrono::steady_clock;

int main() {
    fmt::print(fmt::fg(fmt::color::gold),
        "==================================================\n"
        "    海康 nHostTimeStamp 时间戳验证工具\n"
        "==================================================\n\n"
    );

    // 加载配置
    auto config = static_param::parse_file("hardware.toml");

    camera::CameraConfig cam_config;
    cam_config.use_camera_sn = static_param::get_param<bool>(config, "Camera", "use_camera_sn");
    cam_config.camera_sn = static_param::get_param<std::string>(config, "Camera", "camera_sn");
    cam_config.use_mfs_config = static_param::get_param<bool>(config, "Camera", "use_config_from_file");
    std::string mfs_filename = static_param::get_param<std::string>(config, "Camera", "config_file_path");
    cam_config.mfs_config_path = std::string(CONFIG_DIR) + "/" + mfs_filename;
    cam_config.use_runtime_config = static_param::get_param<bool>(config, "Camera", "use_camera_config");

    auto param_table = static_param::get_param_table(config, "Camera.config");
    for (const auto& [key, value] : param_table) {
        std::visit([&](const auto& v) {
            using T = std::decay_t<decltype(v)>;
            if constexpr (!std::is_same_v<T, std::vector<int64_t>>) {
                cam_config.runtime_params.emplace_back(key, camera::CameraParam(v));
            }
        }, value);
    }

    // 打开相机
    camera::HikCam cam(cam_config);
    cam.open();
    fmt::print(fmt::fg(fmt::color::green), "相机已打开\n\n");

    // 采集数据
    struct TimestampSample {
        int64_t steady_before_us;   // GetImageBuffer 前的 steady_clock (us)
        int64_t steady_after_us;    // GetImageBuffer 后的 steady_clock (us)
        int64_t host_timestamp;     // nHostTimeStamp (原始值)
        uint32_t frame_num;
    };

    std::vector<TimestampSample> samples;
    samples.reserve(100);

    fmt::print("采集 100 帧数据...\n\n");

    for (int i = 0; i < 100; ++i) {
        TimestampSample sample;

        // 记录 GetImageBuffer 前的时间
        auto before = SteadyClock::now();
        sample.steady_before_us = duration_cast<microseconds>(before.time_since_epoch()).count();

        // 直接调用底层 SDK (需要修改 HikCam 暴露接口，这里用 capture 代替)
        cv::Mat& img = cam.capture();

        // 记录 GetImageBuffer 后的时间
        auto after = SteadyClock::now();
        sample.steady_after_us = duration_cast<microseconds>(after.time_since_epoch()).count();

        // 由于 HikCam::capture() 没有暴露 stFrameInfo，我们需要用另一种方式
        // 这里先用 frame_id 作为参考
        sample.frame_num = cam.frame_id;
        sample.host_timestamp = 0;  // 需要修改 HikCam 才能获取

        samples.push_back(sample);

        if ((i + 1) % 20 == 0) {
            fmt::print("  已采集 {} 帧\n", i + 1);
        }
    }

    fmt::print("\n");

    // 分析结果
    fmt::print(fmt::fg(fmt::color::cyan),
        "==================================================\n"
        "    分析结果\n"
        "==================================================\n\n"
    );

    // 计算帧间隔
    std::vector<int64_t> intervals;
    for (size_t i = 1; i < samples.size(); ++i) {
        int64_t interval = samples[i].steady_after_us - samples[i-1].steady_after_us;
        intervals.push_back(interval);
    }

    // 统计
    double mean_interval = 0;
    for (auto v : intervals) mean_interval += v;
    mean_interval /= intervals.size();

    double std_interval = 0;
    for (auto v : intervals) std_interval += (v - mean_interval) * (v - mean_interval);
    std_interval = std::sqrt(std_interval / intervals.size());

    fmt::print("帧间隔统计 (基于 steady_clock):\n");
    fmt::print("  平均: {:.1f} us ({:.1f} Hz)\n", mean_interval, 1e6 / mean_interval);
    fmt::print("  标准差: {:.1f} us\n", std_interval);
    fmt::print("  最小: {} us\n", *std::min_element(intervals.begin(), intervals.end()));
    fmt::print("  最大: {} us\n", *std::max_element(intervals.begin(), intervals.end()));

    // 计算 GetImageBuffer 调用耗时
    std::vector<int64_t> call_durations;
    for (const auto& s : samples) {
        call_durations.push_back(s.steady_after_us - s.steady_before_us);
    }

    double mean_duration = 0;
    for (auto v : call_durations) mean_duration += v;
    mean_duration /= call_durations.size();

    fmt::print("\nGetImageBuffer 调用耗时:\n");
    fmt::print("  平均: {:.1f} us\n", mean_duration);
    fmt::print("  最小: {} us\n", *std::min_element(call_durations.begin(), call_durations.end()));
    fmt::print("  最大: {} us\n", *std::max_element(call_durations.begin(), call_durations.end()));

    fmt::print(fmt::fg(fmt::color::yellow),
        "\n==================================================\n"
        "    注意\n"
        "==================================================\n"
        "当前 HikCam 未暴露 nHostTimeStamp。\n"
        "需要修改 hik_camera.hpp/cpp 才能获取。\n"
        "==================================================\n\n"
    );

    return 0;
}
