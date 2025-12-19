//
// Test Hardware Node (Serial + Camera + Sync)
//

#include <chrono>
#include <thread>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "hardware/hardware_node.hpp"
#include "plugin/debug/logger.hpp"
#include "umt/umt.hpp"

int main() {
    using namespace std::chrono_literals;

    debug::init_session();
    debug::print(debug::PrintMode::INFO, "TestHardware", "Hardware node test starting...");

    // Start hardware node in separate thread
    std::thread(hardware::start_hardware_node).detach();

    // Wait for hardware to initialize
    std::this_thread::sleep_for(500ms);

    // Subscribe to sync frames
    umt::Subscriber<hardware::SyncFrame> sub("sync_frame");

    debug::print(debug::PrintMode::INFO, "TestHardware", "Subscribed to sync_frame, waiting for data...");

    int frame_count = 0;
    int sync_count = 0;
    auto stats_time = std::chrono::steady_clock::now();

    while (true) {
        try {
            // Wait for frame with timeout
            auto frame = sub.pop_for(1000);  // 1 second timeout

            frame_count++;
            if (frame.serial_valid) {
                sync_count++;
            }

            // Print stats every second
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - stats_time).count() >= 1000) {
                auto imu = frame.imu();
                debug::print(debug::PrintMode::INFO, "TestHardware",
                    "FPS: {}, Synced: {}, IMU: yaw={:.2f} pitch={:.2f}",
                    frame_count, sync_count,
                    imu.yaw, imu.pitch);
                frame_count = 0;
                sync_count = 0;
                stats_time = now;
            }

            // Display image
            if (!frame.image.empty()) {
                cv::Mat display;
                cv::resize(frame.image, display, cv::Size(720, 540));

                // Draw IMU info on image
                auto imu = frame.imu();
                std::string info = fmt::format("IMU: yaw={:.1f} pitch={:.1f} valid={}",
                    imu.yaw, imu.pitch, frame.serial_valid ? "Y" : "N");
                cv::putText(display, info, cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

                cv::imshow("Hardware Test", display);
            }

            int key = cv::waitKey(1);
            if (key == 27 || key == 'q') {
                break;
            }

        } catch (const umt::MessageError_Timeout&) {
            debug::print(debug::PrintMode::WARNING, "TestHardware", "Timeout waiting for frame");
        }
    }

    // Signal stop
    auto running = umt::BasicObjManager<bool>::find_or_create("hardware_running", true);
    running->get() = false;

    cv::destroyAllWindows();
    debug::print(debug::PrintMode::INFO, "TestHardware", "Test completed");

    return 0;
}
