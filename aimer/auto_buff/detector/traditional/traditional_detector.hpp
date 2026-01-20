//
// 传统能量机关检测器
// 基于轮廓分析和几何特征的检测方法
//

#ifndef AIMER_AUTOBUFF_TRADITIONAL_DETECTOR_HPP
#define AIMER_AUTOBUFF_TRADITIONAL_DETECTOR_HPP

#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "aimer/auto_buff/detector/common/detector_interface.hpp"

namespace autobuff::detector {

/**
 * @brief 传统检测器配置
 */
struct TraditionalDetectorConfig {
    // 颜色通道差阈值
    int gray_threshold_red = 80;
    int gray_threshold_blue = 80;

    // R标检测参数
    double r_area_min = 100;
    double r_area_max = 5000;
    double r_circularity_min = 0.2;
    double r_circularity_max = 0.9;

    // 靶心检测参数
    double target_area_min = 60;
    double target_area_max = 6000;
    double target_aspect_min = 0.99;
    double target_aspect_max = 1.55;

    // 箭头检测参数
    double arrow_area_min = 100;
    double arrow_area_max = 4000;
    double arrow_aspect_min = 2.0;
    double arrow_aspect_max = 6.0;

    // 准确度阈值
    double accuracy_threshold = 0.8;

    // 调试选项
    bool show_window = false;
    bool save_debug_image = false;
};

/**
 * @brief 传统能量机关检测器
 *
 * 检测流程:
 * 1. 色差二值化
 * 2. 轮廓提取
 * 3. 特征分析 (R标/靶心/箭头)
 * 4. 几何约束过滤
 * 5. 特征关联
 */
class TraditionalDetector : public BuffDetectorInterface {
public:
    explicit TraditionalDetector(const TraditionalDetectorConfig& config = {});

    BuffDetectionResult detect(const cv::Mat& image, double timestamp) override;
    void set_enemy_color(EnemyColor color) override;
    EnemyColor get_enemy_color() const override;
    cv::Mat get_debug_image() const override;

private:
    // ========== 内部结构体 ==========

    // 候选R标
    struct RCenterCandidate {
        cv::Point2f center;
        std::vector<cv::Point> contour;
        cv::RotatedRect rect;
        double area;
        double circularity;
        float confidence;
    };

    // 候选靶心
    struct TargetCandidate {
        cv::Point2f center;
        std::vector<cv::Point> contour;
        std::vector<cv::Point> inner_contours;  // 子轮廓 (用于判断激活状态)
        cv::RotatedRect rect;
        double area;
        double aspect_ratio;
        bool is_active;  // 已击打
        float confidence;
    };

    // 候选箭头
    struct ArrowCandidate {
        cv::Point2f tip;
        cv::Point2f tail;
        cv::Point2f center;
        std::vector<cv::Point> contour;
        cv::RotatedRect rect;
        double area;
        double aspect_ratio;
        double angle;  // 方向角
        float confidence;
    };

    // ========== 检测流程 ==========

    /**
     * @brief 色差二值化
     * diff = (enemy_color == RED) ? R - B : B - R
     * bin = (diff > threshold) ? 255 : 0
     */
    cv::Mat binary_color_diff(const cv::Mat& src);

    /**
     * @brief 查找R标候选
     */
    std::vector<RCenterCandidate> find_r_centers(
        const cv::Mat& binary,
        const std::vector<std::vector<cv::Point>>& contours,
        const std::vector<cv::Vec4i>& hierarchy);

    /**
     * @brief 查找靶心候选
     */
    std::vector<TargetCandidate> find_targets(
        const cv::Mat& binary,
        const std::vector<std::vector<cv::Point>>& contours,
        const std::vector<cv::Vec4i>& hierarchy);

    /**
     * @brief 查找箭头候选
     */
    std::vector<ArrowCandidate> find_arrows(
        const cv::Mat& binary,
        const std::vector<std::vector<cv::Point>>& contours);

    /**
     * @brief 选择最佳R标
     */
    bool select_best_r_center(
        const std::vector<RCenterCandidate>& candidates,
        DetectedRCenter& result);

    /**
     * @brief 过滤并关联靶心
     * 使用R标位置进行几何约束
     */
    void filter_and_match_targets(
        const std::vector<TargetCandidate>& candidates,
        const DetectedRCenter& r_center,
        std::array<DetectedTarget, NUM_SLOTS>& targets);

    /**
     * @brief 选择最佳箭头并关联到靶心
     */
    bool select_best_arrow(
        const std::vector<ArrowCandidate>& candidates,
        const DetectedRCenter& r_center,
        const std::array<DetectedTarget, NUM_SLOTS>& targets,
        DetectedArrow& result,
        int& active_slot_id);

    /**
     * @brief 计算准确度评分
     */
    double compute_accuracy(
        const DetectedRCenter& r_center,
        const std::array<DetectedTarget, NUM_SLOTS>& targets);

    /**
     * @brief 绘制调试图像
     */
    void draw_debug_image(const cv::Mat& src, const BuffDetectionResult& result);

    // ========== 成员变量 ==========

    TraditionalDetectorConfig config_;
    EnemyColor enemy_color_ = EnemyColor::RED;
    cv::Mat debug_image_;
    int frame_count_ = 0;
};

}  // namespace autobuff::detector

#endif  // AIMER_AUTOBUFF_TRADITIONAL_DETECTOR_HPP
