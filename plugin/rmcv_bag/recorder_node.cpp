/**
 * @file recorder_node.cpp
 * @brief 录制节点实现
 */

#include "recorder_node.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <thread>

#include <fmt/format.h>
#include <opencv2/videoio.hpp>

#include "hardware/hardware_node.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/param/static_config.hpp"
#include "plugin/stats/fps_stats.hpp"
#include "umt/umt.hpp"

namespace rmcv_bag {

using SteadyClock = std::chrono::steady_clock;
namespace fs = std::filesystem;

// ============================================================================
// 工具函数
// ============================================================================

/**
 * @brief 生成不重复的文件路径 (自动添加序号)
 * @param dir 目录路径
 * @param basename 基础文件名 (不含扩展名)
 * @param ext 扩展名 (如 ".mkv", ".csv")
 * @return 不重复的文件路径，如 "dir/basename_001.ext"
 */
std::string get_unique_filepath(const std::string& dir,
                                const std::string& basename,
                                const std::string& ext) {
    for (int i = 1; i <= 999; ++i) {
        std::string filepath = fmt::format("{}/{}_{:03d}{}", dir, basename, i, ext);
        if (!fs::exists(filepath)) {
            return filepath;
        }
    }
    // 超过 999 个文件，返回带时间戳的
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::string timestamp = fmt::format("{:%H-%M-%S}", *std::localtime(&time_t_now));
    return fmt::format("{}/{}_{}{}", dir, basename, timestamp, ext);
}

// ============================================================================
// 配置结构体 (启动时加载一次)
// ============================================================================

struct RecorderConfig {
    bool enable_recording = false;  // 是否启用录制
    bool record_raw_video = true;
    bool record_imu_csv = true;
    double camera_fps = 200.0;
    std::string video_codec = "MJPG";
    int64_t sample_interval = 3;  // 采样间隔，1=不跳帧

    double get_record_fps() const {
        return camera_fps / static_cast<double>(sample_interval);
    }

    static RecorderConfig load() {
        RecorderConfig cfg;
        auto table = static_param::parse_file("debugger.toml");
        cfg.enable_recording = static_param::get_param<bool>(table, "Recorder", "enable_recording");
        cfg.record_raw_video = static_param::get_param<bool>(table, "Recorder", "record_raw_video");
        cfg.record_imu_csv = static_param::get_param<bool>(table, "Recorder", "record_imu_csv");
        cfg.camera_fps = static_param::get_param<double>(table, "Recorder", "camera_fps");
        cfg.video_codec = static_param::get_param<std::string>(table, "Recorder", "video_codec");
        cfg.sample_interval = static_param::get_param<int64_t>(table, "Recorder", "sample_interval");
        if (cfg.sample_interval < 1) cfg.sample_interval = 1;
        return cfg;
    }
};

// ============================================================================
// CSV Writer
// ============================================================================

class CsvWriter {
public:
    explicit CsvWriter(const std::string& filepath)
        : file_(filepath, std::ios::out | std::ios::trunc) {
        if (!file_.is_open()) {
            throw std::runtime_error("Failed to open CSV file: " + filepath);
        }
    }

    ~CsvWriter() {
        if (file_.is_open()) {
            file_.close();
        }
    }

    void write_header(const std::vector<std::string>& columns) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (size_t i = 0; i < columns.size(); ++i) {
            file_ << columns[i];
            if (i < columns.size() - 1) file_ << ",";
        }
        file_ << "\n";
    }

    void write_row(const std::vector<std::string>& values) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (size_t i = 0; i < values.size(); ++i) {
            file_ << values[i];
            if (i < values.size() - 1) file_ << ",";
        }
        file_ << "\n";
    }

    void flush() {
        std::lock_guard<std::mutex> lock(mutex_);
        file_.flush();
    }

private:
    std::ofstream file_;
    std::mutex mutex_;
};

// ============================================================================
// Video Writer 包装器
// ============================================================================

class VideoWriterWrapper {
public:
    bool open(const std::string& filepath, double fps, cv::Size frame_size,
              const std::string& codec = "MJPG") {
        if (codec.size() < 4) return false;
        int fourcc = cv::VideoWriter::fourcc(
            codec[0], codec[1], codec[2], codec[3]);
        writer_.open(filepath, fourcc, fps, frame_size);
        return writer_.isOpened();
    }

    void write(const cv::Mat& frame) {
        if (writer_.isOpened()) {
            writer_.write(frame);
        }
    }

    void release() {
        if (writer_.isOpened()) {
            writer_.release();
        }
    }

    bool is_opened() const { return writer_.isOpened(); }

private:
    cv::VideoWriter writer_;
};

// ============================================================================
// Recorder Node
// ============================================================================

void start_recorder_node() {
    debug::print(debug::PrintMode::INFO, "RecorderNode", "Starting recorder node...");

    // 加载配置 (启动时读取一次)
    RecorderConfig config = RecorderConfig::load();

    // 获取比赛模式指针 (循环外获取，循环内只读值)
    auto match_mode_obj = umt::BasicObjManager<bool>::find("match_mode");
    auto is_match_mode = [&]() { return match_mode_obj && match_mode_obj->get(); };

    debug::print(debug::PrintMode::INFO, "RecorderNode",
        "Config: raw={} imu={} fps={:.1f} codec={} sample=1/{}{}",
        config.record_raw_video, config.record_imu_csv,
        config.get_record_fps(), config.video_codec, config.sample_interval,
        is_match_mode() ? " [MATCH MODE - 强制 raw+imu]" : "");

    // 获取会话路径
    std::string session_path = debug::get_session_path();
    if (session_path.empty()) {
        debug::print(debug::PrintMode::ERROR, "RecorderNode",
            "Session path not initialized, call debug::init_session() first");
        return;
    }

    // 订阅消息 (fifo_size=1 只保留最新帧)
    umt::Subscriber<hardware::SyncFrame> sync_sub("sync_frame", 1);

    // 运行状态
    auto running = umt::BasicObjManager<bool>::find_or_create("app_running", true);

    // 录制器资源
    std::unique_ptr<VideoWriterWrapper> raw_writer;
    std::unique_ptr<CsvWriter> imu_writer;

    bool was_recording = false;
    int frame_count = 0;
    int csv_row_count = 0;
    int64_t sample_counter = 0;  // 采样计数器

    stats::FpsStats stats("RecorderNode", "frames");

    debug::print(debug::PrintMode::INFO, "RecorderNode", "Output dir: {}", session_path);

    while (running->get()) {
        try {
            // 检查录制开关 (比赛模式强制录制 raw + imu)
            bool match_mode = is_match_mode();
            bool enable_recording = config.enable_recording || match_mode;

            // ========== 状态转换: 开始录制 ==========
            if (enable_recording && !was_recording) {
                debug::print(debug::PrintMode::INFO, "RecorderNode", "Recording started{}",
                    match_mode ? " [MATCH MODE]" : "");

                // 初始化 CSV writer (比赛模式强制)
                if (config.record_imu_csv || match_mode) {
                    std::string imu_filepath = get_unique_filepath(session_path, "imu", ".csv");
                    imu_writer = std::make_unique<CsvWriter>(imu_filepath);
                    imu_writer->write_header({
                        "timestamp_us", "frame_id",
                        "yaw", "pitch", "roll",
                        "robot_id", "enemy_color",
                        "bullet_speed", "aim_mode", "allow_fire",
                        "serial_timestamp"
                    });
                    csv_row_count = 0;
                }

                // VideoWriter 延迟初始化 (比赛模式强制 raw)
                if (config.record_raw_video || match_mode) {
                    raw_writer = std::make_unique<VideoWriterWrapper>();
                }

                frame_count = 0;
                sample_counter = 0;
                was_recording = true;
            }

            // ========== 状态转换: 停止录制 ==========
            if (!enable_recording && was_recording) {
                debug::print(debug::PrintMode::INFO, "RecorderNode",
                    "Recording stopped: {} frames, {} IMU rows", frame_count, csv_row_count);

                if (raw_writer) {
                    raw_writer->release();
                    raw_writer.reset();
                }
                if (imu_writer) {
                    imu_writer->flush();
                    imu_writer.reset();
                }

                was_recording = false;
            }

            // ========== 获取数据 ==========
            // 尝试获取 sync_frame
            hardware::SyncFrame sync_frame;
            bool has_sync = false;
            try {
                sync_frame = sync_sub.pop_for(50);  // 50ms 超时
                has_sync = true;
            } catch (const umt::MessageError_Timeout&) {
                // 超时，继续
            }

            if (!enable_recording) continue;
            if (!has_sync) continue;

            // 采样判断 (视频和CSV同步)
            sample_counter++;
            bool should_record = (sample_counter % config.sample_interval == 0);
            if (!should_record) continue;

            // 开始计时
            auto write_start = SteadyClock::now();

            // 判断 sync_frame 是否完整 (image + serial 都有效)
            bool sync_valid = !sync_frame.image.empty() && sync_frame.serial_valid;

            // ========== 写入原始视频 ==========
            if (sync_valid && raw_writer) {
                // 延迟初始化 (首帧时获取尺寸)
                if (!raw_writer->is_opened()) {
                    cv::Size frame_size = sync_frame.image.size();
                    std::string filepath = get_unique_filepath(session_path, "raw", ".mkv");
                    if (raw_writer->open(filepath,
                            config.get_record_fps(), frame_size, config.video_codec)) {
                        debug::print(debug::PrintMode::INFO, "RecorderNode",
                            "Raw video: {}x{} @ {:.1f}fps [{}]",
                            frame_size.width, frame_size.height,
                            config.get_record_fps(), config.video_codec);
                    } else {
                        debug::print(debug::PrintMode::ERROR, "RecorderNode",
                            "Failed to open raw video: {}", filepath);
                    }
                }

                if (raw_writer->is_opened()) {
                    raw_writer->write(sync_frame.image);
                    frame_count++;
                }
            }

            // ========== 写入 IMU CSV ==========
            if (sync_valid && imu_writer) {
                const auto& s = sync_frame.serial_data;
                imu_writer->write_row({
                    std::to_string(sync_frame.timestamp_us),
                    std::to_string(sync_frame.frame_id),
                    fmt::format("{:.4f}", s.yaw),
                    fmt::format("{:.4f}", s.pitch),
                    fmt::format("{:.4f}", s.roll),
                    std::to_string(s.robot_id),
                    std::to_string(s.enemy_color),
                    fmt::format("{:.2f}", s.bullet_speed),
                    std::to_string(s.aim_mode),
                    std::to_string(s.allow_fire ? 1 : 0),
                    std::to_string(s.recv_time_us)
                });
                csv_row_count++;

                // 每 100 行 flush 一次，防止断电丢失数据
                if (csv_row_count % 100 == 0) {
                    imu_writer->flush();
                }
            }

            // 计算写入耗时
            auto write_end = SteadyClock::now();
            float latency_ms = std::chrono::duration_cast<std::chrono::microseconds>(
                write_end - write_start).count() / 1000.0f;

            // 更新统计
            stats.update(latency_ms, has_sync);

        } catch (const umt::MessageError_Stopped&) {
            // 发布者断开，等待重连
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } catch (const std::exception& e) {
            debug::print(debug::PrintMode::ERROR, "RecorderNode",
                "Exception: {}", e.what());
        }
    }

    // 清理资源
    if (raw_writer) raw_writer->release();
    if (imu_writer) imu_writer->flush();

    debug::print(debug::PrintMode::INFO, "RecorderNode", "Recorder node stopped");
}

}  // namespace rmcv_bag
