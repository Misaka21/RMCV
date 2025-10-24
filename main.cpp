// C++ system headers
#include <algorithm>
#include <iostream>
#include <optional>
#include <string>

// Third-party library headers

// Project headers
#include "hardware/hik_cam/hik_camera.hpp"
#include "param/static_config.hpp"
#include "param/runtime_parameter.hpp"
#include "plugin/debug/logger.hpp"

int main() {
    debug::init_md_file("log.log");

    camera::HikCam camera;
    camera.open();

    const auto param = static_param::parse_file("test.toml");
    const auto param_file_name = "test.toml";

    fmt::print(fmt::fg(fmt::color::gold), "======================Loading parameters======================\n");

    std::thread([=]() { runtime_param::parameter_run(param_file_name); }).detach();
    runtime_param::wait_for_param("ok");
    auto server_param = static_param::get_param<std::string>(param, "database", "server");

    debug::print(
        "info",
        "test",
        "toml:{}",
        server_param);
    debug::print("log", "param", runtime_param::get_param<std::string>("database.server"));

    debug::print("info", "main", "main_start");
    return 0;
}
