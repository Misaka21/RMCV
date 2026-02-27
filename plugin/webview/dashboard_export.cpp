//
// Dashboard - pybind11 导出
// 仅在 ENABLE_WEBVIEW 启用时编译
//

#ifdef ENABLE_WEBVIEW

#include <pybind11/embed.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "dashboard.hpp"

namespace py = pybind11;

// Value 转 Python 对象
py::object value_to_python(const dashboard::Value& v) {
    return std::visit([](auto&& arg) -> py::object {
        return py::cast(arg);
    }, v);
}

// 获取所有数据为 Python dict
py::dict get_all_as_dict() {
    py::dict result;
    for (const auto& [k, v] : dashboard::all()) {
        result[py::str(k)] = value_to_python(v);
    }
    return result;
}

PYBIND11_EMBEDDED_MODULE(dashboard, m) {
    m.doc() = "Dashboard dynamic data registry";

    // 获取单个值
    m.def("get", [](const std::string& key) {
        return value_to_python(dashboard::get(key));
    }, py::arg("key"), "Get value by key");

    // 获取所有键
    m.def("keys", &dashboard::keys, "Get all registered keys");

    // 获取所有数据
    m.def("all", &get_all_as_dict, "Get all data as dict");

    // 版本号 (每次 set 递增，用于变化检测)
    m.def("version", &dashboard::version, "Get data version counter");

    // 设置值 (主要用于测试)
    m.def("set_int", [](const std::string& k, int v) { dashboard::set(k, v); });
    m.def("set_float", [](const std::string& k, float v) { dashboard::set(k, v); });
    m.def("set_double", [](const std::string& k, double v) { dashboard::set(k, v); });
    m.def("set_bool", [](const std::string& k, bool v) { dashboard::set(k, v); });
    m.def("set_str", [](const std::string& k, const std::string& v) { dashboard::set(k, v); });
}

#endif  // ENABLE_WEBVIEW
