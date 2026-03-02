// 能量机关检测器预处理实现

#include "preprocess.hpp"

#include <algorithm>

#include <opencv2/imgproc.hpp>

namespace autobuff::detector {

std::pair<cv::Mat, LetterboxMeta> letterbox_resize(const cv::Mat& src, int target_size)
{
    LetterboxMeta meta;
    meta.src_w = src.cols;
    meta.src_h = src.rows;
    meta.net_w = target_size;
    meta.net_h = target_size;

    // 等比例缩放，取两个方向缩放比的最小值
    meta.scale = std::min(
        static_cast<float>(target_size) / src.cols,
        static_cast<float>(target_size) / src.rows
    );

    int new_w = static_cast<int>(src.cols * meta.scale);
    int new_h = static_cast<int>(src.rows * meta.scale);

    // 缩放
    cv::Mat resized;
    cv::resize(src, resized, cv::Size(new_w, new_h));

    // 创建灰色背景
    cv::Mat padded(target_size, target_size, CV_8UC3, cv::Scalar(114, 114, 114));

    // 居中放置
    meta.pad_x = static_cast<float>((target_size - new_w) / 2);
    meta.pad_y = static_cast<float>((target_size - new_h) / 2);

    resized.copyTo(padded(cv::Rect(
        static_cast<int>(meta.pad_x),
        static_cast<int>(meta.pad_y),
        new_w, new_h
    )));

    return {padded, meta};
}

}  // namespace autobuff::detector
