//
// Test Serial Communication
//

#include <chrono>
#include <thread>

#include "hardware/serial/serial_thread.hpp"
#include "plugin/debug/logger.hpp"
#include "umt/umt.hpp"

int main() {
    using namespace std::chrono_literals;

    debug::init_session("test_serial");
    debug::print(debug::PrintMode::INFO, "TestSerial", "Serial communication test starting...");

    try {
        // Start serial communication (从配置文件读取)
        serial::start_serial_communication();

        // Get receive queue
        auto recv_queue = umt::BasicObjManager<serial::ReceiveQueue>::find_or_create("receive_queue");

        debug::print(debug::PrintMode::INFO, "TestSerial", "Serial started, monitoring receive queue...");

        // Monitor loop
        int count = 0;
        auto start_time = std::chrono::steady_clock::now();

        while (true) {
            auto& queue = recv_queue->get();

            while (!queue.empty()) {
                auto data = queue.front();
                queue.pop();
                count++;
                if (count%100==0)
                    debug::print(debug::PrintMode::DEBUG, "TestSerial",
                        "[{}] yaw: {:.5f}, pitch: {:.5f}, roll: {:.5f}",
                        count, data.yaw, data.pitch, data.roll);
            }

            // Print stats every second
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
            if (elapsed >= 1) {
                debug::print(debug::PrintMode::INFO, "TestSerial",
                    "Received {} packets in last second", count);
                count = 0;
                start_time = now;
            }

            std::this_thread::sleep_for(1ms);
        }

    } catch (const std::exception& e) {
        debug::print(debug::PrintMode::ERROR, "TestSerial", "Error: {}", e.what());
        return 1;
    }

    return 0;
}
