//
// 传统能量机关检测器
// 基于轮廓分析和几何特征的检测方法
//

#ifndef AIMER_AUTOBUFF_TRADITIONAL_DETECTOR_HPP
#define AIMER_AUTOBUFF_TRADITIONAL_DETECTOR_HPP

#include <array>
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

    // 缺口检测参数
    double gap_area_ratio_min = 0.025;   // 缺口面积/靶心面积 最小比
    double gap_area_ratio_max = 0.20;    // 缺口面积/靶心面积 最大比
    double gap_aspect_min = 1.55;        // 缺口长宽比最小
    double gap_aspect_max = 8.0;         // 缺口长宽比最大

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
 * 4. 缺口角点检测 → 扇叶尖端计算
 * 5. 几何约束过滤
 * 6. 特征关联
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

    // 缺口信息
    struct GapInfo {
        cv::Point2f left_corner;   // 左角点 (叉积法)
        cv::Point2f right_corner;  // 右角点
        cv::Point2f center;        // 缺口中心
        bool valid = false;
    };

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
        cv::RotatedRect ellipse;           // 椭圆拟合
        std::vector<cv::Point> contour;    // 外轮廓
        std::vector<int> gap_indices;      // 缺口轮廓索引
        std::vector<GapInfo> gaps;         // 排序后的缺口 (left_top, right_top, right_bottom, left_bottom)
        double area;
        double aspect_ratio;
        bool is_active;  // 已激打
        float confidence;

        // 扇叶尖端 (top, right, bottom, left)
        std::array<cv::Point2f, 4> fan_tips;
        bool has_fan_tips = false;
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
     * @brief 查找靶心候选 (含缺口检测)
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
     * @brief 检测缺口并计算扇叶尖端
     *
     * 1. 找到4个缺口轮廓
     * 2. 对每个缺口用叉积找左右角点
     * 3. 按角度排序缺口
     * 4. 计算扇叶尖端 = 相邻缺口角点中点
     */
    bool detect_gaps_and_fan_tips(
        TargetCandidate& target,
        const std::vector<std::vector<cv::Point>>& contours,
        const std::vector<cv::Vec4i>& hierarchy);

    /**
     * @brief 选择最佳R标
     */
    bool select_best_r_center(
        const std::vector<RCenterCandidate>& candidates,
        DetectedRCenter& result);

    /**
     * @brief 过滤并关联靶心
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

    // ========== 辅助函数 ==========

    /**
     * @brief 用叉积找轮廓的最左和最右点
     */
    static std::pair<int, int> get_left_right_idx(
        const std::vector<cv::Point2f>& contour,
        const cv::Point2f& center);

    // ========== 成员变量 ==========

    TraditionalDetectorConfig config_;
    EnemyColor enemy_color_ = EnemyColor::RED;
    cv::Mat debug_image_;
    int frame_count_ = 0;
};

}  // namespace autobuff::detector

#endif  // AIMER_AUTOBUFF_TRADITIONAL_DETECTOR_HPP
