#ifdef ENABLE_RERUN

#include "rmcv_rerun.hpp"
#include "collection_adapters.hpp"

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include <fmt/format.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rerun.hpp>

#include "plugin/param/static_config.hpp"
#include "plugin/debug/logger.hpp"

namespace rr {

namespace {

std::unique_ptr<rerun::RecordingStream> g_rec;
std::atomic<bool> g_enabled{false};

// 图像后台编码线程
struct ImageSlot {
    cv::Mat img;        // 最新帧（调用线程写，后台线程读）
    bool fresh = false; // 是否有新帧
};

std::mutex g_img_mutex;
std::condition_variable g_img_cv;
std::unordered_map<std::string, ImageSlot> g_img_slots; // 每个 path 一个槽位
std::thread g_img_thread;
std::atomic<bool> g_img_running{false};

// 后台线程：取最新帧 → resize → encode → log
void image_worker() {
    const std::vector<int> jpeg_params = {cv::IMWRITE_JPEG_QUALITY, 75};

    while (g_img_running.load(std::memory_order_relaxed)) {
        std::unordered_map<std::string, cv::Mat> work;

        {
            std::unique_lock lock(g_img_mutex);
            g_img_cv.wait_for(lock, std::chrono::milliseconds(10), [] {
                for (auto& [_, slot] : g_img_slots) {
                    if (slot.fresh) return true;
                }
                return false;
            });

            // 取出所有新帧
            for (auto& [path, slot] : g_img_slots) {
                if (slot.fresh) {
                    work[path] = std::move(slot.img);
                    slot.fresh = false;
                }
            }
        }

        // 无锁区域：resize + encode + log
        for (auto& [path, img] : work) {
            cv::Mat half;
            cv::resize(img, half, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);

            std::vector<uint8_t> jpeg_buf;
            if (!cv::imencode(".jpg", half, jpeg_buf, jpeg_params)) continue;

            g_rec->log(path, rerun::EncodedImage::from_bytes(
                std::move(jpeg_buf),
                rerun::components::MediaType::jpeg()));
        }
    }
}

}  // namespace

void init() {
    auto cfg = static_param::parse_file("debugger.toml");
    bool enable = static_param::get_param<bool>(cfg, "Rerun", "enable");
    if (!enable) {
        g_enabled.store(false, std::memory_order_relaxed);
        debug::print(debug::PrintMode::INFO, "Rerun", "Disabled by config");
        return;
    }

    std::string mode = static_param::get_param<std::string>(cfg, "Rerun", "mode");
    int64_t port = static_param::get_param<int64_t>(cfg, "Rerun", "port");

    g_rec = std::make_unique<rerun::RecordingStream>("RMCV2026");

    if (mode == "serve") {
        auto result = g_rec->serve_grpc(
            "0.0.0.0",
            static_cast<uint16_t>(port));
        if (result.is_err()) {
            debug::print(debug::PrintMode::ERROR, "Rerun",
                "Failed to start gRPC server: {}", result.error.description);
            g_rec.reset();
            return;
        }
        debug::print(debug::PrintMode::INFO, "Rerun",
            "gRPC server started on port {}", port);
    } else if (mode == "spawn") {
        auto result = g_rec->spawn();
        if (result.is_err()) {
            debug::print(debug::PrintMode::ERROR, "Rerun",
                "Failed to spawn viewer: {}", result.description);
            g_rec.reset();
            return;
        }
        debug::print(debug::PrintMode::INFO, "Rerun", "Spawned viewer");
    } else {
        debug::print(debug::PrintMode::ERROR, "Rerun",
            "Unknown mode: {} (supported: serve, spawn)", mode);
        g_rec.reset();
        return;
    }

    // 启动图像编码后台线程
    g_img_running.store(true, std::memory_order_relaxed);
    g_img_thread = std::thread(image_worker);

    g_enabled.store(true, std::memory_order_relaxed);
}

void shutdown() {
    g_enabled.store(false, std::memory_order_relaxed);

    // 停止图像编码线程
    g_img_running.store(false, std::memory_order_relaxed);
    g_img_cv.notify_all();
    if (g_img_thread.joinable()) {
        g_img_thread.join();
    }
    {
        std::lock_guard lock(g_img_mutex);
        g_img_slots.clear();
    }

    if (g_rec) {
        auto err = g_rec->flush_blocking();
        if (err.is_err()) {
            debug::print(debug::PrintMode::WARNING, "Rerun",
                "Flush failed: {}", err.description);
        }
        g_rec.reset();
    }
    debug::print(debug::PrintMode::INFO, "Rerun", "Shutdown complete");
}

bool enabled() {
    return g_enabled.load(std::memory_order_relaxed);
}

void scalar(const std::string& path, double value) {
    if (!g_enabled.load(std::memory_order_relaxed)) return;
    g_rec->log(path, rerun::Scalars(value));
}

void scalar(const std::string& path, int value) {
    if (!g_enabled.load(std::memory_order_relaxed)) return;
    g_rec->log(path, rerun::Scalars(static_cast<double>(value)));
}

void scalar(const std::string& path, bool value) {
    if (!g_enabled.load(std::memory_order_relaxed)) return;
    g_rec->log(path, rerun::Scalars(value ? 1.0 : 0.0));
}

void set_time(double timestamp_s) {
    if (!g_enabled.load(std::memory_order_relaxed)) return;
    g_rec->set_time_duration_secs("timestamp", timestamp_s);
}

void set_time_us(int64_t timestamp_us) {
    if (!g_enabled.load(std::memory_order_relaxed)) return;
    g_rec->set_time_duration_secs("timestamp", timestamp_us / 1e6);
}

void image(const std::string& path, const cv::Mat& img, int skip_factor) {
    if (!g_enabled.load(std::memory_order_relaxed)) return;
    if (img.empty()) return;

    if (skip_factor <= 0) skip_factor = 1;

    // 跳帧检查仍在调用线程（几乎零开销）
    thread_local std::unordered_map<std::string, int> counters;
    auto& count = counters[path];
    if (++count % skip_factor != 0) return;

    // clone 丢进槽位，后台线程做 resize + encode
    // 1440x1080x3 clone ≈ 0.5ms，可接受
    {
        std::lock_guard lock(g_img_mutex);
        auto& slot = g_img_slots[path];
        slot.img = img.clone();
        slot.fresh = true;
    }
    g_img_cv.notify_one();
}

void points3d(const std::string& path,
              const std::vector<Eigen::Vector3d>& positions,
              uint8_t r, uint8_t g, uint8_t b,
              float radius) {
    if (!g_enabled.load(std::memory_order_relaxed)) return;
    if (positions.empty()) return;

    std::vector<rerun::Position3D> pts;
    pts.reserve(positions.size());
    for (const auto& p : positions) {
        pts.push_back(rerun::Position3D(
            static_cast<float>(p.x()),
            static_cast<float>(p.y()),
            static_cast<float>(p.z())));
    }

    g_rec->log(path, rerun::Points3D(pts)
        .with_colors({rerun::Color(r, g, b)})
        .with_radii({radius}));
}

void arrows3d(const std::string& path,
              const std::vector<Eigen::Vector3d>& origins,
              const std::vector<Eigen::Vector3d>& vectors,
              uint8_t r, uint8_t g, uint8_t b) {
    if (!g_enabled.load(std::memory_order_relaxed)) return;
    if (origins.empty()) return;

    std::vector<rerun::Position3D> org;
    std::vector<rerun::Vector3D> vec;
    org.reserve(origins.size());
    vec.reserve(vectors.size());

    for (const auto& o : origins) {
        org.push_back(rerun::Position3D(
            static_cast<float>(o.x()),
            static_cast<float>(o.y()),
            static_cast<float>(o.z())));
    }
    for (const auto& v : vectors) {
        vec.push_back(rerun::Vector3D(
            static_cast<float>(v.x()),
            static_cast<float>(v.y()),
            static_cast<float>(v.z())));
    }

    g_rec->log(path, rerun::Arrows3D::from_vectors(vec)
        .with_origins(org)
        .with_colors({rerun::Color(r, g, b)}));
}

void text(const std::string& path, const std::string& body) {
    if (!g_enabled.load(std::memory_order_relaxed)) return;
    g_rec->log(path, rerun::TextLog(body));
}

}  // namespace rr

#endif  // ENABLE_RERUN
