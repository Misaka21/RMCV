//
// Test Serial Communication
//

#include <chrono>
#include <thread>

#include "hardware/serial/serial_thread.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/param/static_config.hpp"
#include "umt/umt.hpp"

int main() {
    using namespace std::chrono_literals;

    debug::init_session();
    debug::print(debug::PrintMode::INFO, "TestSerial", "Serial communication test starting...");

    try {
        // Load config
        auto config = static_param::parse_file("hardware.toml");
        std::string port_name = static_param::get_param<std::string>(config, "Serial", "port_name");
        int64_t baudrate = static_param::get_param<int64_t>(config, "Serial", "baudrate");

        debug::print(debug::PrintMode::INFO, "TestSerial", "Port: {}, Baudrate: {}", port_name, baudrate);

        // Start serial communication
        serial::start_serial_communication(port_name, static_cast<int>(baudrate));

        // Get receive queue
        auto recv_queue = umt::BasicObjManager<serial::ReceiveQueue>::find_or_create("receive_queue");

        debug::print(debug::PrintMode::LOG, "TestSerial", "Serial started, monitoring receive queue...");

        // Monitor loop
        int count = 0;
        auto start_time = std::chrono::steady_clock::now();

        while (true) {
            auto& queue = recv_queue->get();

            while (!queue.empty()) {
                auto data = queue.front();
                queue.pop();
                count++;

                debug::print(debug::PrintMode::DEBUG, "TestSerial",
                    "[{}] yaw: {:.2f}, pitch: {:.2f}, dist: {:.2f}",
                    count, data.yaw, data.pitch, data.distance);
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
