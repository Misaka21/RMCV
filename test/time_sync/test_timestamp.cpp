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

    // ========================================
    // 5. 详细抖动分析
    // ========================================
    fmt::print(fmt::fg(fmt::color::cyan),
        "\n==================================================\n"
        "    详细抖动分析\n"
        "==================================================\n\n"
    );

    // 5.1 分析 offset 是否随时间漂移 (判断是否同源)
    fmt::print("5.1 时钟漂移分析:\n");
    {
        // 将数据分成 10 段，看每段的平均 offset 是否有趋势
        int segment_size = samples.size() / 10;
        std::vector<double> segment_means;
        for (int seg = 0; seg < 10; ++seg) {
            double sum = 0;
            int start = seg * segment_size;
            int end = (seg + 1) * segment_size;
            for (int i = start; i < end; ++i) {
                sum += offsets[i];
            }
            segment_means.push_back(sum / segment_size);
        }

        // 计算漂移 (最后一段 - 第一段)
        double drift = segment_means.back() - segment_means.front();
        double drift_rate = drift / (mean_steady_diff * samples.size() / 1e6);  // us/s

        fmt::print("  各段平均 offset (us):\n    ");
        for (int i = 0; i < 10; ++i) {
            fmt::print("{:.1f} ", segment_means[i] - mean_offset);  // 相对于总平均
        }
        fmt::print("\n");
        fmt::print("  总漂移: {:.1f} us (采集期间)\n", drift);
        fmt::print("  漂移率: {:.2f} us/s = {:.1f} ppm\n", drift_rate, drift_rate);

        if (std::abs(drift_rate) < 10) {
            fmt::print(fmt::fg(fmt::color::green), "  → 漂移很小，时钟同源\n");
        } else if (std::abs(drift_rate) < 100) {
            fmt::print(fmt::fg(fmt::color::yellow), "  → 有轻微漂移，可能是晶振精度差异\n");
        } else {
            fmt::print(fmt::fg(fmt::color::red), "  → 漂移明显，可能不同源\n");
        }
    }

    // 5.2 分析抖动来源：GetImageBuffer 延迟 vs offset 的相关性
    fmt::print("\n5.2 抖动来源分析:\n");
    {
        // 计算 GetImageBuffer 延迟与 offset 的相关系数
        std::vector<double> durations_f(call_durations.begin(), call_durations.end());

        // 归一化
        double mean_dur = mean_duration;
        double mean_off = mean_offset;

        double cov = 0, var_dur = 0, var_off = 0;
        for (size_t i = 0; i < samples.size(); ++i) {
            double d = durations_f[i] - mean_dur;
            double o = offsets[i] - mean_off;
            cov += d * o;
            var_dur += d * d;
            var_off += o * o;
        }
        double corr = cov / std::sqrt(var_dur * var_off);

        fmt::print("  GetImageBuffer延迟 vs offset 相关系数: {:.3f}\n", corr);

        if (std::abs(corr) > 0.7) {
            fmt::print(fmt::fg(fmt::color::yellow),
                "  → 强相关! 抖动主要来自 GetImageBuffer 调用时机不确定\n");
            fmt::print("    建议: host_timestamp 应在 GetImageBuffer 返回时刻附近\n");
        } else if (std::abs(corr) > 0.3) {
            fmt::print(fmt::fg(fmt::color::yellow),
                "  → 中等相关，GetImageBuffer 延迟贡献部分抖动\n");
        } else {
            fmt::print("  → 弱相关，抖动可能来自其他因素\n");
        }

        // 5.3 使用 steady_before 代替 steady_mid 计算 offset
        fmt::print("\n5.3 不同参考点的 offset 标准差:\n");

        // 用 before
        std::vector<double> offsets_before;
        for (const auto& s : samples) {
            double host_us = s.host_timestamp / scale;
            offsets_before.push_back(host_us - s.steady_before_us);
        }
        double mean_before = std::accumulate(offsets_before.begin(), offsets_before.end(), 0.0) / offsets_before.size();
        double std_before = 0;
        for (auto v : offsets_before) std_before += (v - mean_before) * (v - mean_before);
        std_before = std::sqrt(std_before / offsets_before.size());

        // 用 after
        std::vector<double> offsets_after;
        for (const auto& s : samples) {
            double host_us = s.host_timestamp / scale;
            offsets_after.push_back(host_us - s.steady_after_us);
        }
        double mean_after = std::accumulate(offsets_after.begin(), offsets_after.end(), 0.0) / offsets_after.size();
        double std_after = 0;
        for (auto v : offsets_after) std_after += (v - mean_after) * (v - mean_after);
        std_after = std::sqrt(std_after / offsets_after.size());

        fmt::print("  使用 steady_before 作参考: std = {:.1f} us\n", std_before);
        fmt::print("  使用 steady_mid 作参考:    std = {:.1f} us\n", std_offset);
        fmt::print("  使用 steady_after 作参考:  std = {:.1f} us\n", std_after);

        // 找出最小的
        double min_std = std::min({std_before, std_offset, std_after});
        if (min_std == std_before) {
            fmt::print(fmt::fg(fmt::color::green),
                "  → host_timestamp 更接近 GetImageBuffer 调用前\n");
        } else if (min_std == std_after) {
            fmt::print(fmt::fg(fmt::color::green),
                "  → host_timestamp 更接近 GetImageBuffer 返回后\n");
        } else {
            fmt::print(fmt::fg(fmt::color::green),
                "  → host_timestamp 在 GetImageBuffer 调用中间\n");
        }

        // 5.4 帧间隔抖动分析
        fmt::print("\n5.4 帧间隔抖动分析:\n");
        double std_host_diff = 0;
        for (auto v : host_diffs) std_host_diff += (v - mean_host_diff) * (v - mean_host_diff);
        std_host_diff = std::sqrt(std_host_diff / host_diffs.size());

        double std_steady_diff = 0;
        for (auto v : steady_diffs) std_steady_diff += (v - mean_steady_diff) * (v - mean_steady_diff);
        std_steady_diff = std::sqrt(std_steady_diff / steady_diffs.size());

        // host_diff 转换为 us
        double std_host_diff_us = std_host_diff / scale;

        fmt::print("  host_timestamp 帧间隔标准差: {:.1f} (原始单位) = {:.1f} us\n",
            std_host_diff, std_host_diff_us);
        fmt::print("  steady_clock 帧间隔标准差:   {:.1f} us\n", std_steady_diff);

        if (std_host_diff_us < std_steady_diff * 0.5) {
            fmt::print(fmt::fg(fmt::color::green),
                "  → host_timestamp 帧间隔更稳定，说明是硬件触发时间戳\n");
        } else if (std_host_diff_us > std_steady_diff * 2) {
            fmt::print(fmt::fg(fmt::color::red),
                "  → host_timestamp 帧间隔抖动更大，可能不是精确时间戳\n");
        } else {
            fmt::print("  → 两者帧间隔抖动相近\n");
        }
    }

    // 5.5 结论
    fmt::print(fmt::fg(fmt::color::cyan),
        "\n==================================================\n"
        "    结论\n"
        "==================================================\n\n"
    );
    fmt::print("GetImageBuffer 调用延迟抖动: {:.1f} us\n", std_duration);
    fmt::print("offset 总抖动:              {:.1f} us\n", std_offset);
    double unexplained = std::sqrt(std::max(0.0, std_offset * std_offset - std_duration * std_duration / 4));
    fmt::print("除延迟外的残余抖动:         {:.1f} us (估算)\n", unexplained);
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
