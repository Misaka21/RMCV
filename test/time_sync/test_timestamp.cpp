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
#include <algorithm>
#include <numeric>

#include <fmt/format.h>
#include <fmt/color.h>

#include "hardware/hik_cam/hik_camera.hpp"
#include "plugin/param/static_config.hpp"
#include "plugin/debug/logger.hpp"

using namespace std::chrono;
using SteadyClock = std::chrono::steady_clock;

int main() {
    fmt::print(fmt::fg(fmt::color::gold),
        "==================================================\n"
        "    海康 nHostTimeStamp 时间戳验证工具\n"
        "==================================================\n\n"
    );

    // 初始化日志
    debug::init_session("test_timestamp");

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
    constexpr int NUM_SAMPLES = 2000;
    samples.reserve(NUM_SAMPLES);

    fmt::print("采集 {} 帧数据...\n\n", NUM_SAMPLES);

    for (int i = 0; i < NUM_SAMPLES; ++i) {
        TimestampSample sample;

        // 记录 GetImageBuffer 前的时间
        auto before = SteadyClock::now();
        sample.steady_before_us = duration_cast<microseconds>(before.time_since_epoch()).count();

        // 捕获图像
        cv::Mat& img = cam.capture();
        (void)img;

        // 记录 GetImageBuffer 后的时间
        auto after = SteadyClock::now();
        sample.steady_after_us = duration_cast<microseconds>(after.time_since_epoch()).count();

        // 获取 SDK 时间戳
        sample.frame_num = cam.frame_id;
        sample.host_timestamp = cam.host_timestamp;

        samples.push_back(sample);

        if ((i + 1) % 50 == 0) {
            fmt::print("  已采集 {} 帧\n", i + 1);
        }
    }

    fmt::print("\n");

    // ========================================
    // 分析结果
    // ========================================
    fmt::print(fmt::fg(fmt::color::cyan),
        "==================================================\n"
        "    分析结果\n"
        "==================================================\n\n"
    );

    // 1. 打印前几个样本的原始数据
    fmt::print("前5帧原始数据:\n");
    fmt::print("{:>6} {:>18} {:>18} {:>18}\n",
        "Frame", "steady_before(us)", "host_timestamp", "steady_after(us)");
    for (int i = 0; i < 5 && i < static_cast<int>(samples.size()); ++i) {
        const auto& s = samples[i];
        fmt::print("{:>6} {:>18} {:>18} {:>18}\n",
            s.frame_num, s.steady_before_us, s.host_timestamp, s.steady_after_us);
    }
    fmt::print("\n");

    // 2. 判断 nHostTimeStamp 的单位
    // 比较相邻帧的差值
    std::vector<int64_t> host_diffs;
    std::vector<int64_t> steady_diffs;
    for (size_t i = 1; i < samples.size(); ++i) {
        host_diffs.push_back(samples[i].host_timestamp - samples[i-1].host_timestamp);
        steady_diffs.push_back(samples[i].steady_after_us - samples[i-1].steady_after_us);
    }

    double mean_host_diff = std::accumulate(host_diffs.begin(), host_diffs.end(), 0.0) / host_diffs.size();
    double mean_steady_diff = std::accumulate(steady_diffs.begin(), steady_diffs.end(), 0.0) / steady_diffs.size();

    fmt::print("帧间隔分析:\n");
    fmt::print("  steady_clock 帧间隔平均: {:.1f} us ({:.1f} Hz)\n",
        mean_steady_diff, 1e6 / mean_steady_diff);
    fmt::print("  host_timestamp 帧间隔平均: {:.1f}\n", mean_host_diff);

    // 推断单位
    double ratio = mean_host_diff / mean_steady_diff;
    fmt::print("\n单位推断:\n");
    fmt::print("  host_diff / steady_diff = {:.3f}\n", ratio);

    std::string unit_guess;
    double scale = 1.0;
    if (ratio > 500 && ratio < 2000) {
        unit_guess = "纳秒 (ns)";
        scale = 1000.0;  // ns -> us
        fmt::print("  推测 nHostTimeStamp 单位: {} (ratio ≈ 1000)\n", unit_guess);
    } else if (ratio > 0.5 && ratio < 2.0) {
        unit_guess = "微秒 (us)";
        scale = 1.0;
        fmt::print("  推测 nHostTimeStamp 单位: {} (ratio ≈ 1)\n", unit_guess);
    } else if (ratio > 0.0005 && ratio < 0.002) {
        unit_guess = "毫秒 (ms)";
        scale = 0.001;  // ms -> us
        fmt::print("  推测 nHostTimeStamp 单位: {} (ratio ≈ 0.001)\n", unit_guess);
    } else {
        unit_guess = "未知";
        fmt::print("  推测 nHostTimeStamp 单位: {} (ratio = {:.3f})\n", unit_guess, ratio);
    }

    // 3. 验证是否同源
    fmt::print("\n同源性验证:\n");

    // 计算 host_timestamp (转换为us) 和 steady_clock 的差值
    std::vector<double> offsets;
    for (const auto& s : samples) {
        double host_us = s.host_timestamp / scale;  // 转换为 us
        double steady_mid = (s.steady_before_us + s.steady_after_us) / 2.0;
        offsets.push_back(host_us - steady_mid);
    }

    double mean_offset = std::accumulate(offsets.begin(), offsets.end(), 0.0) / offsets.size();
    double std_offset = 0;
    for (auto v : offsets) std_offset += (v - mean_offset) * (v - mean_offset);
    std_offset = std::sqrt(std_offset / offsets.size());

    fmt::print("  host_timestamp - steady_clock 偏移:\n");
    fmt::print("    平均: {:.1f} us\n", mean_offset);
    fmt::print("    标准差: {:.1f} us\n", std_offset);

    // 如果标准差很小（<100us），说明同源
    if (std_offset < 100) {
        fmt::print(fmt::fg(fmt::color::green),
            "\n结论: nHostTimeStamp 和 steady_clock 是同源的!\n");
        fmt::print("  标准差 {:.1f} us < 100 us，可以直接用于时间同步。\n\n", std_offset);
    } else if (std_offset < 500) {
        fmt::print(fmt::fg(fmt::color::yellow),
            "\n结论: nHostTimeStamp 和 steady_clock 可能同源，但抖动较大。\n");
        fmt::print("  标准差 {:.1f} us，建议使用 steady_clock。\n\n", std_offset);
    } else {
        fmt::print(fmt::fg(fmt::color::red),
            "\n结论: nHostTimeStamp 和 steady_clock 可能不同源!\n");
        fmt::print("  标准差 {:.1f} us 过大，建议使用 steady_clock。\n\n", std_offset);
    }

    // 4. 计算 GetImageBuffer 调用耗时
    std::vector<int64_t> call_durations;
    for (const auto& s : samples) {
        call_durations.push_back(s.steady_after_us - s.steady_before_us);
    }

    double mean_duration = std::accumulate(call_durations.begin(), call_durations.end(), 0.0) / call_durations.size();
    double std_duration = 0;
    for (auto v : call_durations) std_duration += (v - mean_duration) * (v - mean_duration);
    std_duration = std::sqrt(std_duration / call_durations.size());

    fmt::print("GetImageBuffer 调用耗时:\n");
    fmt::print("  平均: {:.1f} us ({:.2f} ms)\n", mean_duration, mean_duration / 1000.0);
    fmt::print("  标准差: {:.1f} us\n", std_duration);
    fmt::print("  最小: {} us\n", *std::min_element(call_durations.begin(), call_durations.end()));
    fmt::print("  最大: {} us\n", *std::max_element(call_durations.begin(), call_durations.end()));

    fmt::print("\n");

    // 记录分析结果到日志
    debug::print("info", "Timestamp", "========== 时间戳验证结果 ==========");
    debug::print("info", "Timestamp", "帧率: {:.1f} Hz", 1e6 / mean_steady_diff);
    debug::print("info", "Timestamp", "nHostTimeStamp 单位: {}", unit_guess);
    debug::print("info", "Timestamp", "host_diff / steady_diff = {:.3f}", ratio);
    debug::print("info", "Timestamp", "偏移平均: {:.1f} us", mean_offset);
    debug::print("info", "Timestamp", "偏移标准差: {:.1f} us", std_offset);

    if (std_offset < 100) {
        debug::print("info", "Timestamp", "结论: nHostTimeStamp 和 steady_clock 同源 (标准差 < 100us)");
    } else if (std_offset < 500) {
        debug::print("warning", "Timestamp", "结论: nHostTimeStamp 和 steady_clock 可能同源，但抖动较大 (标准差 = {:.1f}us)", std_offset);
    } else {
        debug::print("warning", "Timestamp", "结论: nHostTimeStamp 和 steady_clock 可能不同源 (标准差 = {:.1f}us)", std_offset);
    }

    debug::print("info", "Timestamp", "GetImageBuffer 调用耗时: 平均 {:.1f}us, 标准差 {:.1f}us", mean_duration, std_duration);
    debug::print("info", "Timestamp", "==========================================");

    return 0;
}
