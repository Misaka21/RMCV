//
// Test Camera Capture
//

#include <chrono>
#include <thread>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "hardware/hik_cam/hik_camera.hpp"
#include "plugin/debug/logger.hpp"

int main() {
    using namespace std::chrono_literals;

    debug::init_session();
    debug::print(debug::PrintMode::INFO, "TestCamera", "Camera test starting...");

    try {
        camera::HikCam cam;
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
