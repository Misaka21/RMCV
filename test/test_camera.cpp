//
// Test Camera Capture
//

#include <chrono>
#include <thread>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "hardware/hik_cam/hik_camera.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/param/static_config.hpp"

/**
 * @brief Load camera configuration from TOML
 */
camera::CameraConfig load_camera_config(const toml::table& config) {
    camera::CameraConfig cam_config;

    // Device selection
    cam_config.use_camera_sn = static_param::get_param<bool>(config, "Camera", "use_camera_sn");
    cam_config.camera_sn = static_param::get_param<std::string>(config, "Camera", "camera_sn");

    // MFS config file
    cam_config.use_mfs_config = static_param::get_param<bool>(config, "Camera", "use_config_from_file");
    std::string mfs_filename = static_param::get_param<std::string>(config, "Camera", "config_file_path");
    cam_config.mfs_config_path = std::string(CONFIG_DIR) + "/" + mfs_filename;

    // Runtime parameters
    cam_config.use_runtime_config = static_param::get_param<bool>(config, "Camera", "use_camera_config");

    // Get Camera.config table and convert to CameraParam
    auto param_table = static_param::get_param_table(config, "Camera.config");
    for (const auto& [key, value] : param_table) {
        std::visit([&](const auto& v) {
            using T = std::decay_t<decltype(v)>;
            // Skip vector types (not supported by camera API)
            if constexpr (!std::is_same_v<T, std::vector<int64_t>>) {
                cam_config.runtime_params.emplace_back(key, camera::CameraParam(v));
            }
        }, value);
    }

    return cam_config;
}

int main() {
    using namespace std::chrono_literals;

    debug::init_session();
    debug::print(debug::PrintMode::INFO, "TestCamera", "Camera test starting...");

    try {
        // Load config and create camera
        auto config = static_param::parse_file("hardware.toml");
        camera::CameraConfig cam_config = load_camera_config(config);
        camera::HikCam cam(cam_config);
        cam.open();

        debug::print(debug::PrintMode::LOG, "TestCamera", "Camera opened, starting capture...");

        int fps_count = 0;
        auto fps_time = std::chrono::steady_clock::now();

        while (true) {
            cv::Mat& img = cam.capture();
            if (img.empty()) {
                debug::print(debug::PrintMode::WARNING, "TestCamera", "Empty frame");
                continue;
            }

            fps_count++;

            // Show FPS stats every second
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - fps_time).count() >= 1000) {
                debug::print(debug::PrintMode::INFO, "TestCamera",
                    "FPS: {}, Resolution: {}x{}", fps_count, img.cols, img.rows);
                fps_count = 0;
                fps_time = now;
            }

            // Display image (optional, comment out if no display)
            cv::Mat display;
            cv::resize(img, display, cv::Size(720, 540));
            cv::imshow("Camera Test", display);

            int key = cv::waitKey(1);
            if (key == 27 || key == 'q') {  // ESC or 'q' to quit
                break;
            }
        }

        cv::destroyAllWindows();

    } catch (const std::exception& e) {
        debug::print(debug::PrintMode::ERROR, "TestCamera", "Error: {}", e.what());
        return 1;
    }

    debug::print(debug::PrintMode::INFO, "TestCamera", "Test completed");
    return 0;
}
