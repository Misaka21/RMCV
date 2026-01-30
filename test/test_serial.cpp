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

        // 订阅串口数据 (使用 Message 系统)
        umt::Subscriber<serial::SerialReceiveData> subscriber("serial_receive");

        debug::print(debug::PrintMode::INFO, "TestSerial", "Serial started, monitoring receive...");

        // Monitor loop
        int count = 0;
        auto start_time = std::chrono::steady_clock::now();

        while (true) {
            try {
                auto data = subscriber.pop_for(100);  // 100ms 超时
                count++;
                if (count % 100 == 0) {
                    debug::print(debug::PrintMode::DEBUG, "TestSerial",
                        "[{}] yaw: {:.5f}, pitch: {:.5f}, roll: {:.5f}",
                        count, data.yaw, data.pitch, data.roll);
                }
            } catch (const umt::MessageError_Timeout&) {
                // 超时，继续
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
        }

    } catch (const std::exception& e) {
        debug::print(debug::PrintMode::ERROR, "TestSerial", "Error: {}", e.what());
        return 1;
    }

    return 0;
}
