//
// 传统能量机关检测器 (2026)
// - 基于颜色通道差分 + 轮廓层次分析的纯 CV 管线
// - 不含神经网络，适应大幅非激活场景
//

#ifndef AIMER_AUTOBUFF_DETECTOR_TRADITIONAL_HPP
#define AIMER_AUTOBUFF_DETECTOR_TRADITIONAL_HPP

#include <vector>

#include <opencv2/core.hpp>

#include "aimer/auto_buff/detector/common/detector_interface.hpp"

namespace autobuff::detector {

/**
 * @brief 传统能量机关检测器
 *
 * 使用颜色差分二值化 + 轮廓层次分析检测 R 标中心和扇叶装甲板。
 *
 * 同步检测器，不依赖 GPU。
 */
class TraditionalBuffDetector : public BuffDetectorInterface {
public:
    TraditionalBuffDetector();

    BuffDetectionResult detect(const cv::Mat& image, double timestamp) override;
    void set_enemy_color(EnemyColor color) override { enemy_color_ = color; }
    EnemyColor get_enemy_color() const override { return enemy_color_; }
    cv::Mat get_debug_image() const override;
    bool is_async() const override { return false; }

    /** 启用/关闭调试绘制 (关闭时 detect() 不产生调试图像，节省 CPU) */
    void set_debug_enabled(bool on) { debug_enabled_ = on; }

private:
    EnemyColor enemy_color_ = EnemyColor::UNKNOWN;
    bool debug_enabled_ = false;
    cv::Mat debug_image_;

    // ---- 管线阶段 (debug_img 非空时绘制中间调试信息) ----

    /** 颜色通道差分 + 阈值二值化 */
    cv::Mat preprocess(const cv::Mat& bgr, bool use_red) const;

    /** 颜色校验：在原图中确认轮廓属于敌方颜色，标记无效轮廓 */
    void filter_by_color(
        const cv::Mat& bgr,
        const std::vector<std::vector<cv::Point>>& contours,
        std::vector<bool>& used,
        cv::Mat& debug_img) const;

    /** 检测 R 标中心 */
    bool find_r_center(
        const std::vector<std::vector<cv::Point>>& contours,
        const std::vector<cv::Vec4i>& hierarchy,
        const std::vector<bool>& used,
        cv::Size image_size,
        DetectedRCenter& out,
        cv::Mat& debug_img) const;

    /** 检测扇叶靶心 */
    std::vector<DetectedTarget> find_targets(
        const std::vector<std::vector<cv::Point>>& contours,
        const std::vector<cv::Vec4i>& hierarchy,
        std::vector<bool>& used,
        cv::Mat& debug_img) const;

    /** 根据 R 中心分配槽位并填充 keypoints */
    void assign_slots(
        std::vector<DetectedTarget>& targets,
        const DetectedRCenter& r_center) const;

    // ---- 辅助 ----

    static int find_top_parent(int idx, const std::vector<cv::Vec4i>& hierarchy);

    /** 以 R 中心为参考，对扇叶角点排序 (左上-右上-右下-左下) */
    static void order_corners(
        std::vector<cv::Point2f>& corners,
        const cv::Point2f& fan_center,
        const cv::Point2f& r_center);
};

}  // namespace autobuff::detector

#endif  // AIMER_AUTOBUFF_DETECTOR_TRADITIONAL_HPP
