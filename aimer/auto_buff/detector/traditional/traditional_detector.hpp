// 传统能量机关检测器 (2026)
// 基于轮廓 + 几何约束，迁移 rm_vision_core 的 Active/Inactive 思路:
// - Lit(被点亮) 目标: 同心环/十环特征 (active target)
// - Unlit(未点亮) 目标: 具有 4 个缺口 (inactive target)，使用椭圆矫正提取 gap 角点

#ifndef AIMER_AUTOBUFF_TRADITIONAL_DETECTOR_HPP
#define AIMER_AUTOBUFF_TRADITIONAL_DETECTOR_HPP

#include <array>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

#include "aimer/auto_buff/detector/common/detector_interface.hpp"

namespace autobuff::detector {

struct TraditionalDetectorConfig {
    // 颜色通道差阈值
    int gray_threshold_red = 80;   // R - B
    int gray_threshold_blue = 80;  // B - R

    // R 标检测
    double r_area_min = 100.0;
    double r_area_max = 5000.0;
    double r_circularity_min = 0.2;
    double r_circularity_max = 0.9;

    // 靶心外轮廓 (通用)
    double target_area_min = 60.0;
    double target_area_max = 6000.0;
    double target_aspect_min = 0.99;
    double target_aspect_max = 1.55;

    // Lit(Active) 目标: 子轮廓面积比阈值
    double active_min_sub_area_ratio = 0.10;
    double active_max_total_sub_area_ratio = 0.65;  // 十环过滤

    // Unlit(Inactive) gap 参数 (相对 outer area)
    double gap_area_ratio_min = 0.025;
    double gap_area_ratio_max = 0.20;
    double gap_aspect_min = 1.55;
    double gap_aspect_max = 8.0;

    // gap 椭圆矫正后的几何约束 (比例基于 correction_radius)
    double gap_min_center_dist_ratio = 0.10;
    double gap_max_center_dist_ratio = 0.90;
    double gap_circle_radius_ratio = 0.85;  // 将角点投影到圆上的比例

    // 调试选项
    bool show_window = false;
    bool save_debug_image = false;
};

class TraditionalDetector final : public BuffDetectorInterface {
public:
    explicit TraditionalDetector(const TraditionalDetectorConfig& config = {});

    BuffDetectionResult detect(const cv::Mat& image, double timestamp) override;
    void set_enemy_color(EnemyColor color) override { enemy_color_ = color; }
    EnemyColor get_enemy_color() const override { return enemy_color_; }
    cv::Mat get_debug_image() const override { return debug_image_; }

private:
    struct Gap {
        cv::Point2f left_corner{};
        cv::Point2f right_corner{};
        cv::Point2f center{};
        bool valid = false;
    };

    struct RCenterCandidate {
        cv::Point2f center{};
        std::vector<cv::Point> contour;
        cv::RotatedRect rect;
        double area = 0.0;
        double circularity = 0.0;
        float confidence = 0.f;
    };

    struct TargetCandidate {
        int outer_idx = -1;
        cv::Point2f center{};
        std::vector<cv::Point> contour;   // outer contour
        cv::RotatedRect ellipse;
        double area = 0.0;
        double aspect_ratio = 0.0;

        bool is_lit = false;              // 2026: 被点亮(可打)
        float confidence = 0.f;

        std::vector<int> child_indices;   // hierarchy child indices
        std::vector<Gap> gaps;            // only for unlit
        std::array<cv::Point2f, 4> fan_tips{}; // top/right/bottom/left (unlit)
        bool has_fan_tips = false;
    };

    // ========== pipeline ==========
    cv::Mat binary_color_diff(const cv::Mat& src) const;

    std::vector<RCenterCandidate> find_r_centers(
        const std::vector<std::vector<cv::Point>>& contours,
        const std::vector<cv::Vec4i>& hierarchy) const;

    std::vector<TargetCandidate> find_targets(
        const std::vector<std::vector<cv::Point>>& contours,
        const std::vector<cv::Vec4i>& hierarchy) const;

    bool select_best_r_center(const std::vector<RCenterCandidate>& cands, DetectedRCenter& out) const;

    void assign_targets_to_slots(
        const std::vector<TargetCandidate>& cands,
        const DetectedRCenter& r_center,
        std::array<DetectedTarget, NUM_SLOTS>& out_targets) const;

    // ========== active / inactive ==========
    static bool is_top_level_with_child(const std::vector<cv::Vec4i>& hierarchy, int idx);
    static void collect_all_sub_contours(const std::vector<cv::Vec4i>& hierarchy, int idx, std::vector<int>& out);

    bool is_active_lit_target(
        const std::vector<std::vector<cv::Point>>& contours,
        const std::vector<cv::Vec4i>& hierarchy,
        int outer_idx,
        double outer_area) const;

    bool is_inactive_unlit_target(
        const std::vector<std::vector<cv::Point>>& contours,
        const std::vector<cv::Vec4i>& hierarchy,
        int outer_idx,
        double outer_area) const;

    bool detect_gaps_and_fan_tips(
        TargetCandidate& target,
        const std::vector<std::vector<cv::Point>>& contours,
        const std::vector<cv::Vec4i>& hierarchy) const;

    // ========== rm_vision_core style gap extraction ==========
    static cv::Matx33f get_ellipse_correction_mat(const cv::RotatedRect& ellipse, float& radius);
    static bool is_point_in_ellipse(const cv::Point2f& p, const cv::RotatedRect& ellipse);
    static std::pair<int, int> get_left_right_idx(const std::vector<cv::Point2f>& contour, const cv::Point2f& center);

    bool make_gap(
        const std::vector<cv::Point>& gap_contour,
        const cv::RotatedRect& outer_ellipse,
        double outer_area,
        Gap& out_gap) const;

    // ========== slot matching (72deg grid) ==========
    static double normalize_angle_0_2pi(double a);
    static int solve_best_rotate_deg(const std::vector<double>& angles_deg);

    // ========== debug ==========
    void draw_debug_image(const cv::Mat& src, const BuffDetectionResult& result);

private:
    TraditionalDetectorConfig config_;
    EnemyColor enemy_color_ = EnemyColor::RED;
    mutable cv::Mat debug_image_;
};

}  // namespace autobuff::detector

#endif  // AIMER_AUTOBUFF_TRADITIONAL_DETECTOR_HPP
