/**
 * @file playback_node.cpp
 * @brief 回放节点实现
 */

#include "playback_node.hpp"

#include <chrono>
#include <fstream>
#include <sstream>
#include <thread>
#include <vector>

#include <fmt/format.h>
#include <opencv2/videoio.hpp>

#include "hardware/hardware_node.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/stats/fps_stats.hpp"
#include "umt/umt.hpp"

namespace rmcv_bag {

using SteadyClock = std::chrono::steady_clock;
using hardware::SyncFrame;

// ============================================================================
// CSV Reader
// ============================================================================

// 新协议: 删除 robot_id, enemy_color, allow_fire
struct ImuRecord {
    int64_t timestamp_us;
    int frame_id;
    float yaw;
    float pitch;
    float roll;
    float bullet_speed;
    uint8_t aim_mode;
    bool aiming_lock;
    int64_t serial_timestamp;
};

class CsvReader {
public:
    explicit CsvReader(const std::string& filepath) {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open CSV file: " + filepath);
        }

        std::string line;
        // 跳过 header
        std::getline(file, line);

        // 读取所有行
        while (std::getline(file, line)) {
            ImuRecord record;
            if (parse_line(line, record)) {
                records_.push_back(record);
            }
        }
    }

    size_t size() const { return records_.size(); }

    const ImuRecord& get(size_t index) const {
        return records_.at(index);
    }

private:
    bool parse_line(const std::string& line, ImuRecord& record) {
        std::stringstream ss(line);
        std::string token;
        std::vector<std::string> tokens;

        while (std::getline(ss, token, ',')) {
            tokens.push_back(token);
        }

        if (tokens.size() < 9) return false;

        try {
            record.timestamp_us = std::stoll(tokens[0]);
            record.frame_id = std::stoi(tokens[1]);
            record.yaw = std::stof(tokens[2]);
            record.pitch = std::stof(tokens[3]);
            record.roll = std::stof(tokens[4]);
            record.bullet_speed = std::stof(tokens[5]);
            record.aim_mode = static_cast<uint8_t>(std::stoi(tokens[6]));
            record.aiming_lock = (std::stoi(tokens[7]) != 0);
            record.serial_timestamp = std::stoll(tokens[8]);
            return true;
        } catch (...) {
            return false;
        }
    }

    std::vector<ImuRecord> records_;
};

// ============================================================================
// Playback Node
// ============================================================================

void start_playback_node(const std::string& bag_path, double playback_speed) {
    debug::print(debug::PrintMode::INFO, "PlaybackNode",
        "Starting playback from: {}", bag_path);

    // 打开视频文件
    std::string video_path = bag_path + "/raw.mkv";
    cv::VideoCapture cap(video_path);
    if (!cap.isOpened()) {
        debug::print(debug::PrintMode::ERROR, "PlaybackNode",
            "Failed to open video: {}", video_path);
        return;
    }

    double video_fps = cap.get(cv::CAP_PROP_FPS);
    int total_frames = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));
    debug::print(debug::PrintMode::INFO, "PlaybackNode",
        "Video: {} frames @ {:.1f}fps", total_frames, video_fps);

    // 加载 IMU 数据
    std::string csv_path = bag_path + "/imu.csv";
    std::unique_ptr<CsvReader> csv_reader;
    try {
        csv_reader = std::make_unique<CsvReader>(csv_path);
        debug::print(debug::PrintMode::INFO, "PlaybackNode",
            "IMU CSV: {} records", csv_reader->size());
    } catch (const std::exception& e) {
        debug::print(debug::PrintMode::WARNING, "PlaybackNode",
            "No IMU data: {}", e.what());
    }

    // 设置 UMT
    umt::Publisher<SyncFrame> pub("sync_frame");
    auto running = umt::BasicObjManager<bool>::find_or_create("app_running", true);

    // 计算帧间隔
    double frame_interval_ms = 1000.0 / video_fps;
    if (playback_speed > 0) {
        frame_interval_ms /= playback_speed;
    }

    stats::FpsStats stats("PlaybackNode", "frames");
    int frame_id = 0;
    size_t csv_index = 0;

    debug::print(debug::PrintMode::INFO, "PlaybackNode",
        "Playback speed: {}x, interval: {:.1f}ms",
        playback_speed, frame_interval_ms);

    while (running->get()) {
        auto frame_start = SteadyClock::now();

        // 读取视频帧
        cv::Mat image;
        if (!cap.read(image)) {
            // 视频结束，退出
            debug::print(debug::PrintMode::INFO, "PlaybackNode", "Playback finished");
            break;
        }

        // 构建 SyncFrame
        SyncFrame sync_frame;
        sync_frame.image = image;
        sync_frame.frame_id = frame_id;

        // 填充 IMU 数据 (必须有 CSV)
        if (!csv_reader || csv_reader->size() == 0) {
            throw std::runtime_error("CSV data is required for playback");
        }

        // 使用当前索引或最后一条记录
        size_t idx = std::min(csv_index, csv_reader->size() - 1);
        const auto& imu = csv_reader->get(idx);

        // 使用 CSV 中的原始时间戳 (微秒)
        sync_frame.timestamp_us = imu.timestamp_us;

        sync_frame.serial_data.yaw = imu.yaw;
        sync_frame.serial_data.pitch = imu.pitch;
        sync_frame.serial_data.roll = imu.roll;
        sync_frame.serial_data.bullet_speed = imu.bullet_speed;
        sync_frame.serial_data.aim_mode = imu.aim_mode;
        sync_frame.serial_data.aiming_lock = imu.aiming_lock;
        sync_frame.serial_data.recv_time_us = imu.serial_timestamp;
        sync_frame.serial_valid = true;
        if (csv_index < csv_reader->size()) {
            csv_index++;
        }

        // 发布
        pub.push(sync_frame);
        frame_id++;

        // 统计
        auto frame_end = SteadyClock::now();
        float latency = std::chrono::duration_cast<std::chrono::microseconds>(
            frame_end - frame_start).count() / 1000.0f;
        stats.update(latency, true);

        // 控制播放速度
        if (playback_speed > 0) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                frame_end - frame_start).count();
            int sleep_ms = static_cast<int>(frame_interval_ms - elapsed);
            if (sleep_ms > 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
            }
        }
    }

    debug::print(debug::PrintMode::INFO, "PlaybackNode", "Playback stopped");
}

}  // namespace rmcv_bag
