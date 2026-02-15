// Fallback camera backend for platforms without HikRobot MVS SDK.
// Provides the same camera::HikCam API using OpenCV VideoCapture.

#include "hik_camera.hpp"

#include <chrono>
#include <cstdlib>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace camera {

namespace {

int parse_device_index_from_env() {
    const char* env = std::getenv("RMCV_CAMERA_INDEX");
    if (!env || !*env) return 0;
    try {
        return std::stoi(std::string(env));
    } catch (...) {
        return 0;
    }
}

}  // namespace

HikCam::HikCam(const CameraConfig& config) : _config(config) {
    debug::print(debug::PrintMode::DEBUG, "Camera",
        "HikCam fallback created (OpenCV VideoCapture), use_camera_sn={}", _config.use_camera_sn);
}

HikCam::HikCam() : _config() {
    debug::print(debug::PrintMode::DEBUG, "Camera", "HikCam fallback created with default config");
}

HikCam::~HikCam() = default;

void HikCam::open() {
    if (cap_.isOpened()) return;

    int device_index = parse_device_index_from_env();
    if (!cap_.open(device_index)) {
        throw std::runtime_error(
            "OpenCV VideoCapture failed to open device index " + std::to_string(device_index) +
            " (set env RMCV_CAMERA_INDEX to choose another index).");
    }

    debug::print(debug::PrintMode::INFO, "Camera",
        "HikCam fallback opened via OpenCV VideoCapture (device_index={})", device_index);
}

auto HikCam::capture() -> cv::Mat& {
    cv::Mat frame_bgr;
    cap_ >> frame_bgr;
    if (frame_bgr.empty()) {
        _srcImage.release();
        return _srcImage;
    }

    // Project convention: downstream uses RGB images.
    cv::cvtColor(frame_bgr, _srcImage, cv::COLOR_BGR2RGB);

    frame_id++;
    host_timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()
    ).count();

    return _srcImage;
}

}  // namespace camera

