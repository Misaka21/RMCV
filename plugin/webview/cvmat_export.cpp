//
// cv::Mat 的 pybind11 导出
// 用于 Web 可视化图像流
// 仅在 ENABLE_WEBVIEW 启用时编译
//

#ifdef ENABLE_WEBVIEW

#include <opencv2/opencv.hpp>
#include <pybind11/embed.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "umt/umt.hpp"

namespace py = pybind11;

// cv::Mat 转 numpy array (BGR -> RGB 可选)
inline py::array_t<uint8_t> cvmat_to_nparray(const cv::Mat& mat, bool to_rgb = false) {
    if (mat.empty()) {
        return py::array_t<uint8_t>();
    }

    cv::Mat src = mat;
    if (to_rgb && mat.channels() == 3) {
        cv::cvtColor(mat, src, cv::COLOR_BGR2RGB);
    }

    cv::Mat continuous = src.isContinuous() ? src : src.clone();

    std::vector<ssize_t> shape;
    std::vector<ssize_t> strides;

    if (continuous.channels() == 1) {
        shape = {continuous.rows, continuous.cols};
        strides = {static_cast<ssize_t>(continuous.step[0]), 1};
    } else {
        shape = {continuous.rows, continuous.cols, continuous.channels()};
        strides = {
            static_cast<ssize_t>(continuous.step[0]),
            static_cast<ssize_t>(continuous.channels()),
            1
        };
    }

    // 复制数据，确保 Python 端安全
    auto result = py::array_t<uint8_t>(shape, strides);
    std::memcpy(result.mutable_data(), continuous.data,
                continuous.total() * continuous.elemSize());
    return result;
}

// cv::Mat 转 JPEG bytes
inline py::bytes cvmat_to_jpeg(const cv::Mat& mat, int quality = 80) {
    if (mat.empty()) {
        return py::bytes("");
    }

    std::vector<uchar> buf;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, quality};
    cv::imencode(".jpg", mat, buf, params);

    return py::bytes(reinterpret_cast<char*>(buf.data()), buf.size());
}

// UMT Message 导出 cv::Mat
UMT_EXPORT_MESSAGE_ALIAS(cvMat, cv::Mat, c) {
    c.def("get_nparray", [](const cv::Mat& self, bool to_rgb) {
        return cvmat_to_nparray(self, to_rgb);
    }, py::arg("to_rgb") = false, "Convert to numpy array");

    c.def("to_jpeg", [](const cv::Mat& self, int quality) {
        return cvmat_to_jpeg(self, quality);
    }, py::arg("quality") = 80, "Convert to JPEG bytes");

    c.def("rows", [](const cv::Mat& self) { return self.rows; });
    c.def("cols", [](const cv::Mat& self) { return self.cols; });
    c.def("channels", [](const cv::Mat& self) { return self.channels(); });
    c.def("empty", [](const cv::Mat& self) { return self.empty(); });
    c.def("shape", [](const cv::Mat& self) {
        if (self.channels() == 1) {
            return py::make_tuple(self.rows, self.cols);
        } else {
            return py::make_tuple(self.rows, self.cols, self.channels());
        }
    });
}

#endif  // ENABLE_WEBVIEW
