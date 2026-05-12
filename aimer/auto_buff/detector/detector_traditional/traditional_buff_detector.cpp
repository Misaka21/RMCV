//
// 传统能量机关检测器实现
// 传统 CV 大幅识别管线:
//   颜色通道差分 → 二值化 → 轮廓层次分析 → R 标筛选 → 扇叶聚类 → 槽位分配
//

#include "traditional_buff_detector.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "plugin/param/runtime_parameter.hpp"

namespace autobuff::detector {

using autobuff::NUM_SLOTS;

// ============================================================================
// 构造
// ============================================================================

TraditionalBuffDetector::TraditionalBuffDetector() = default;

// ============================================================================
// 主检测入口
// ============================================================================

BuffDetectionResult TraditionalBuffDetector::detect(
    const cv::Mat& image, double timestamp) {

    BuffDetectionResult result;
    result.timestamp = timestamp;
    result.enemy_color = enemy_color_;
    result.backend = autobuff::DetectorBackend::TRADITIONAL;

    if (image.empty() || enemy_color_ == EnemyColor::UNKNOWN) {
        return result;
    }

    // 调试图像 (仅 debug_enabled_ 时构建)
    if (debug_enabled_) {
        debug_image_ = image.clone();
    }

    const bool use_red = (enemy_color_ == EnemyColor::RED);

    // ---- 1. 颜色差分 + 二值化 ----
    cv::Mat bin = preprocess(image, use_red);

    // 二值图缩略图
    if (debug_enabled_ && !debug_image_.empty()) {
        cv::Mat bin_color;
        cv::cvtColor(bin, bin_color, cv::COLOR_GRAY2BGR);
        double thumb_scale = 0.25;
        cv::Mat thumb;
        cv::resize(bin_color, thumb, {}, thumb_scale, thumb_scale);
        cv::Rect roi(5, 5, thumb.cols, thumb.rows);
        if (roi.x + roi.width <= debug_image_.cols &&
            roi.y + roi.height <= debug_image_.rows) {
            thumb.copyTo(debug_image_(roi));
            cv::rectangle(debug_image_, roi, cv::Scalar(0, 255, 0), 1);
        }
    }

    // ---- 2. 轮廓提取 ----
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(bin, contours, hierarchy,
                     cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        return result;
    }

    // ---- 3. 颜色校验 (蓝=有效/敌方色, 红=被过滤) ----
    std::vector<bool> used(contours.size(), false);
    filter_by_color(image, contours, used, debug_image_);

    // ---- 4. R 标检测 (绿=候选, 红=选中, 黄=图像中心) ----
    DetectedRCenter r_center;
    find_r_center(contours, hierarchy, used, image.size(), r_center, debug_image_);
    result.r_center = r_center;

    // ---- 5. 扇叶检测 (候选点+聚类+角点) ----
    auto targets = find_targets(contours, hierarchy, used, debug_image_);

    // ---- 6. 槽位分配 ----
    assign_slots(targets, r_center);

    // ---- 7. 填充结果 ----
    for (auto& t : targets) {
        if (!t.valid || t.slot_id < 0 || t.slot_id >= NUM_SLOTS) continue;
        if (result.targets[t.slot_id].valid) continue;
        result.targets[t.slot_id] = std::move(t);
    }

    result.update_summary();

    // ---- 8. 最终结果绘制 ----
    if (debug_enabled_ && !debug_image_.empty()) {
        // R 标: 红色实心圆 + 填角矩形
        if (r_center.valid) {
            cv::circle(debug_image_, r_center.center, 10,
                       cv::Scalar(0, 0, 255), -1);
            cv::circle(debug_image_, r_center.center, 12,
                       cv::Scalar(0, 0, 255), 2);
            if (r_center.landmarks.size() >= 4) {
                for (size_t i = 0; i < 4; ++i) {
                    cv::line(debug_image_,
                             r_center.landmarks[i],
                             r_center.landmarks[(i + 1) % 4],
                             cv::Scalar(0, 0, 255), 2);
                }
            }
        }

        // 扇叶: 黄色四边形 + 白色中心 + 绿色槽位号 + 红色角点编号
        for (int i = 0; i < NUM_SLOTS; ++i) {
            if (!result.targets[i].valid) continue;
            const auto& t = result.targets[i];

            // 填充半透明四边形
            if (t.landmarks.size() >= 3) {
                std::vector<std::vector<cv::Point>> poly = {
                    std::vector<cv::Point>(t.landmarks.begin(), t.landmarks.end())
                };
                cv::polylines(debug_image_, poly, true,
                              cv::Scalar(0, 255, 255), 2);
            }

            // 中心点
            cv::circle(debug_image_, t.center, 5,
                       cv::Scalar(255, 255, 255), -1);

            // 槽位号
            cv::putText(debug_image_, std::to_string(t.slot_id),
                        t.center + cv::Point2f(12, -8),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8,
                        cv::Scalar(0, 255, 0), 2);

            // 角点编号 (红点 + 绿字)
            for (size_t j = 0; j < t.landmarks.size(); ++j) {
                cv::circle(debug_image_, t.landmarks[j], 3,
                           cv::Scalar(0, 0, 255), -1);
                cv::putText(debug_image_, std::to_string(j),
                            t.landmarks[j] + cv::Point2f(5, -5),
                            cv::FONT_HERSHEY_SIMPLEX, 0.4,
                            cv::Scalar(0, 255, 0), 1);
            }
        }

        // 状态文本 (左下角)
        std::string status_text = "R=" + std::string(r_center.valid ? "Y" : "N") +
            " T=" + std::to_string(result.target_count) +
            " L=" + std::to_string(result.lit_count);
        cv::putText(debug_image_, status_text,
                    cv::Point(10, debug_image_.rows - 15),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(255, 255, 255), 1);
        cv::imshow("buff_traditional", debug_image_);
        cv::waitKey(1);
    }

    return result;
}

cv::Mat TraditionalBuffDetector::get_debug_image() const {
    return debug_image_;
}

// ============================================================================
// 预处理: 颜色通道差分 + 阈值
// ============================================================================

cv::Mat TraditionalBuffDetector::preprocess(
    const cv::Mat& bgr, bool use_red) const {

    std::vector<cv::Mat> channels;
    cv::split(bgr, channels);

    cv::Mat diff;
    if (use_red) {
        cv::subtract(channels[2], channels[0], diff);  // R - B
    } else {
        cv::subtract(channels[0], channels[2], diff);  // B - R
    }

    int bin_thresh = static_cast<int>(runtime_param::get_param<int64_t>(
        "AutoBuff.Detector.Traditional.bin_threshold"));

    cv::Mat bin;
    cv::threshold(diff, bin, bin_thresh, 255, cv::THRESH_BINARY);
    return bin;
}

// ============================================================================
// 颜色校验 (蓝=敌方色保留, 红=非敌方色过滤)
// ============================================================================

void TraditionalBuffDetector::filter_by_color(
    const cv::Mat& bgr,
    const std::vector<std::vector<cv::Point>>& contours,
    std::vector<bool>& used,
    cv::Mat& debug_img) const {

    const bool filter_red = (enemy_color_ == EnemyColor::RED);
    const int diff_thresh = static_cast<int>(runtime_param::get_param<int64_t>(
        "AutoBuff.Detector.Traditional.color_diff_threshold"));

    for (size_t i = 0; i < contours.size(); ++i) {
        cv::Rect r = cv::boundingRect(contours[i]);
        if (r.width < 5 || r.height < 5) continue;

        cv::Rect rr = r & cv::Rect(0, 0, bgr.cols, bgr.rows);
        if (rr.width < 2 || rr.height < 2) continue;

        const cv::Scalar avg = cv::mean(bgr(rr));
        const double B = avg[0], R = avg[2];
        const double diff_rb = R - B;

        bool invalid = false;
        if (filter_red) {
            if (diff_rb > diff_thresh) invalid = true;
        } else {
            if (-diff_rb > diff_thresh) invalid = true;
        }

        // invalid=true(敌方色) → used=false → 保留
        // invalid=false(非敌方色) → used=true → 过滤
        used[i] = !invalid;

        // 调试绘制: 保留轮廓蓝色, 过滤轮廓红色
        if (!debug_img.empty()) {
            cv::Scalar color = invalid ? cv::Scalar(255, 0, 0)   // 蓝=保留
                                       : cv::Scalar(0, 0, 255);  // 红=过滤
            cv::drawContours(debug_img, contours, static_cast<int>(i),
                             color, invalid ? 2 : 1);
        }
    }
}

// ============================================================================
// R 标中心检测 (绿=候选框, 红=选中, 黄=图像中心)
// ============================================================================

bool TraditionalBuffDetector::find_r_center(
    const std::vector<std::vector<cv::Point>>& contours,
    const std::vector<cv::Vec4i>& hierarchy,
    const std::vector<bool>& used,
    cv::Size image_size,
    DetectedRCenter& out,
    cv::Mat& debug_img) const {

    double min_area = runtime_param::get_param<double>(
        "AutoBuff.Detector.Traditional.rune_center_min_area");
    double max_area = runtime_param::get_param<double>(
        "AutoBuff.Detector.Traditional.rune_center_max_area");
    double ratio_tol = runtime_param::get_param<double>(
        "AutoBuff.Detector.Traditional.rune_center_aspect_ratio_tol");
    double fill_min = runtime_param::get_param<double>(
        "AutoBuff.Detector.Traditional.rune_center_fill_ratio_min");

    struct Candidate {
        cv::Point2f center;
        int idx;
        cv::RotatedRect rr;
    };

    std::vector<Candidate> candidates;

    for (int i = 0; i < static_cast<int>(contours.size()); ++i) {
        if (used[i]) continue;
        if (hierarchy[i][3] != -1) continue;

        double area = cv::contourArea(contours[i]);
        if (area < min_area || area > max_area) continue;

        cv::RotatedRect rr = cv::minAreaRect(contours[i]);
        float w = rr.size.width, h = rr.size.height;
        if (w < 5 || h < 5) continue;

        double ratio = (w > h) ? (w / h) : (h / w);
        if (ratio - 1.0 > ratio_tol) continue;

        double rect_area = w * h;
        if (rect_area <= 1e-5) continue;

        double fill = area / rect_area;
        if (fill < fill_min) continue;

        candidates.push_back({rr.center, i, rr});

        // 调试: 候选框绿色
        if (!debug_img.empty()) {
            cv::Point2f pts[4];
            rr.points(pts);
            for (int k = 0; k < 4; ++k) {
                cv::line(debug_img, pts[k], pts[(k + 1) % 4],
                         cv::Scalar(0, 255, 0), 1);
            }
        }
    }

    if (candidates.empty()) return false;

    // 选择最接近图像中心的候选
    cv::Point2f img_center(image_size.width * 0.5f, image_size.height * 0.5f);

    // 调试: 图像中心黄点
    if (!debug_img.empty()) {
        cv::circle(debug_img, img_center, 5,
                   cv::Scalar(0, 255, 255), -1);
    }

    double best_dist2 = 1e18;
    cv::RotatedRect best_rr;
    double best_fill = 0.0;

    for (auto& c : candidates) {
        double dx = c.center.x - img_center.x;
        double dy = c.center.y - img_center.y;
        double d2 = dx * dx + dy * dy;
        if (d2 < best_dist2) {
            best_dist2 = d2;
            best_rr = c.rr;
            double area = cv::contourArea(contours[c.idx]);
            best_fill = area / (c.rr.size.width * c.rr.size.height);
        }
    }

    out.center = best_rr.center;
    out.valid = true;

    cv::Point2f pts[4];
    best_rr.points(pts);
    out.landmarks.assign(pts, pts + 4);
    out.confidence = std::min(0.95f, 0.5f + 0.5f * static_cast<float>(best_fill));

    // 调试: 选中 R 标红色框 (加粗)
    if (!debug_img.empty()) {
        for (int k = 0; k < 4; ++k) {
            cv::line(debug_img, pts[k], pts[(k + 1) % 4],
                     cv::Scalar(0, 0, 255), 2);
        }
    }

    return true;
}

// ============================================================================
// 扇叶靶心检测
// ============================================================================

int TraditionalBuffDetector::find_top_parent(
    int idx, const std::vector<cv::Vec4i>& hierarchy) {

    int p = hierarchy[idx][3];
    while (p != -1 && hierarchy[p][3] != -1) {
        p = hierarchy[p][3];
    }
    return p;
}

std::vector<DetectedTarget> TraditionalBuffDetector::find_targets(
    const std::vector<std::vector<cv::Point>>& contours,
    const std::vector<cv::Vec4i>& hierarchy,
    std::vector<bool>& used,
    cv::Mat& debug_img) const {

    std::vector<DetectedTarget> results;
    if (hierarchy.empty()) return results;

    double min_area = runtime_param::get_param<double>(
        "AutoBuff.Detector.Traditional.rune_target_min_area");
    double max_area = runtime_param::get_param<double>(
        "AutoBuff.Detector.Traditional.rune_target_max_area");
    double max_sq_ratio = runtime_param::get_param<double>(
        "AutoBuff.Detector.Traditional.rune_target_max_square_ratio");
    double cluster_radius = runtime_param::get_param<double>(
        "AutoBuff.Detector.Traditional.rune_target_cluster_radius");

    // ---- 收集候选 ----
    struct Candidate {
        int idx;
        cv::Point2f center;
        int parent_top;
    };

    std::vector<Candidate> candidates;

    for (int i = 0; i < static_cast<int>(contours.size()); ++i) {
        if (used[i]) continue;

        double area = cv::contourArea(contours[i]);
        if (area < min_area || area > max_area) continue;

        cv::Moments m = cv::moments(contours[i]);
        if (std::abs(m.m00) < 1e-9) continue;

        cv::Point2f ctr(static_cast<float>(m.m10 / m.m00),
                        static_cast<float>(m.m01 / m.m00));
        int tp = find_top_parent(i, hierarchy);
        candidates.push_back({i, ctr, tp});

        // 调试: 候选质心小圆点
        if (!debug_img.empty()) {
            cv::circle(debug_img, ctr, 2,
                       cv::Scalar(255, 200, 0), -1);
        }
    }

    if (candidates.size() < 3) return results;

    // ---- 按顶层父轮廓分组 ----
    std::unordered_map<int, std::vector<int>> groups;
    for (int i = 0; i < static_cast<int>(candidates.size()); ++i) {
        groups[candidates[i].parent_top].push_back(i);
    }

    // 不同聚类用不同颜色
    const std::array<cv::Scalar, 6> cluster_colors = {{
        {255, 100, 100}, {100, 255, 100}, {100, 100, 255},
        {255, 255, 100}, {255, 100, 255}, {100, 255, 255}
    }};

    for (auto& [parent_top, idx_list] : groups) {
        int M = static_cast<int>(idx_list.size());
        if (M < 3 || M > 7) continue;

        // BFS 空间聚类
        std::vector<int> cluster_id(M, -1);
        int cluster_count = 0;

        for (int i = 0; i < M; ++i) {
            if (cluster_id[i] != -1) continue;

            cluster_id[i] = cluster_count;
            std::queue<int> q;
            q.push(i);

            while (!q.empty()) {
                int u = q.front(); q.pop();

                for (int v = 0; v < M; ++v) {
                    if (cluster_id[v] != -1) continue;

                    const auto& cu = candidates[idx_list[u]].center;
                    const auto& cv = candidates[idx_list[v]].center;
                    double dx = cu.x - cv.x;
                    double dy = cu.y - cv.y;
                    if (std::sqrt(dx * dx + dy * dy) <= cluster_radius) {
                        cluster_id[v] = cluster_count;
                        q.push(v);
                    }
                }
            }
            cluster_count++;
        }

        // 统计各簇大小
        std::vector<int> cluster_size(cluster_count, 0);
        for (int id : cluster_id) cluster_size[id]++;

        std::vector<std::vector<cv::Point2f>> cluster_pts(cluster_count);

        for (int i = 0; i < M; ++i) {
            int cid = cluster_id[i];
            if (cluster_size[cid] >= 3) {
                used[candidates[idx_list[i]].idx] = true;
                cluster_pts[cid].push_back(candidates[idx_list[i]].center);
            }
        }

        // 对每个有效簇提取扇叶
        for (int cid = 0; cid < cluster_count; ++cid) {
            if (cluster_pts[cid].size() < 3) continue;

            cv::RotatedRect rr = cv::minAreaRect(cluster_pts[cid]);
            double w = rr.size.width, h = rr.size.height;
            if (w < 1 || h < 1) continue;

            double ratio = (w > h) ? (w / h) : (h / w);
            if (ratio > max_sq_ratio) continue;

            // 调试: 聚类包围框 + 簇内点连线
            if (!debug_img.empty()) {
                auto color = cluster_colors[cid % cluster_colors.size()];
                cv::Point2f rr_pts[4];
                rr.points(rr_pts);
                for (int k = 0; k < 4; ++k) {
                    cv::line(debug_img, rr_pts[k], rr_pts[(k + 1) % 4],
                             color, 1);
                }
                // 簇内点连线到中心
                for (const auto& p : cluster_pts[cid]) {
                    cv::line(debug_img, p, rr.center, color, 1);
                    cv::circle(debug_img, p, 3, color, -1);
                }
            }

            // 提取距簇中心最远的 4 个点作为角点
            std::vector<std::pair<double, cv::Point2f>> dist_list;
            dist_list.reserve(cluster_pts[cid].size());
            for (const auto& p : cluster_pts[cid]) {
                double dx = p.x - rr.center.x;
                double dy = p.y - rr.center.y;
                dist_list.emplace_back(dx * dx + dy * dy, p);
            }
            std::sort(dist_list.begin(), dist_list.end(),
                      [](auto& a, auto& b) { return a.first > b.first; });

            std::vector<cv::Point2f> corners;
            for (int k = 0; k < 4 && k < static_cast<int>(dist_list.size()); ++k) {
                corners.push_back(dist_list[k].second);
            }

            if (corners.size() < 4) continue;

            DetectedTarget t;
            t.center = rr.center;
            t.landmarks = corners;
            t.valid = true;
            t.is_lit = true;
            t.confidence = 0.7f;
            results.push_back(std::move(t));
        }
    }

    return results;
}

// ============================================================================
// 槽位分配 & 角点排序
// ============================================================================

void TraditionalBuffDetector::order_corners(
    std::vector<cv::Point2f>& corners,
    const cv::Point2f& fan_center,
    const cv::Point2f& r_center) {

    if (corners.size() != 4) return;

    cv::Point2f ref_vec = r_center - fan_center;
    float ref_angle = std::atan2(ref_vec.y, ref_vec.x);

    struct Node {
        float ang;
        cv::Point2f p;
    };

    std::vector<Node> arr;
    arr.reserve(4);

    for (const auto& p : corners) {
        cv::Point2f v = p - fan_center;
        float ang = std::atan2(v.y, v.x) - ref_angle;
        while (ang <= -static_cast<float>(CV_PI)) ang += 2 * static_cast<float>(CV_PI);
        while (ang > static_cast<float>(CV_PI))   ang -= 2 * static_cast<float>(CV_PI);
        arr.push_back({ang, p});
    }

    std::sort(arr.begin(), arr.end(),
              [](const Node& a, const Node& b) { return a.ang < b.ang; });

    cv::Point2f lu, ru, rd, ld;
    bool has_lu = false, has_ru = false, has_rd = false, has_ld = false;

    for (const auto& n : arr) {
        if (n.ang > CV_PI / 2 && n.ang <= CV_PI) {
            lu = n.p; has_lu = true;
        } else if (n.ang > 0 && n.ang <= CV_PI / 2) {
            ru = n.p; has_ru = true;
        } else if (n.ang > -CV_PI / 2 && n.ang <= 0) {
            rd = n.p; has_rd = true;
        } else {
            ld = n.p; has_ld = true;
        }
    }

    std::array<cv::Point2f, 4> ordered;

    if (has_lu && has_ru && has_rd && has_ld) {
        ordered = {lu, ru, rd, ld};
    } else {
        float target = 3.0f * static_cast<float>(CV_PI) / 4.0f;
        int best_idx = 0;
        float best_diff = std::numeric_limits<float>::max();
        for (int i = 0; i < static_cast<int>(arr.size()); ++i) {
            float d = std::abs(arr[i].ang - target);
            if (d > static_cast<float>(CV_PI))
                d = 2 * static_cast<float>(CV_PI) - d;
            if (d < best_diff) {
                best_diff = d;
                best_idx = i;
            }
        }
        for (int i = 0; i < 4; ++i) {
            ordered[i] = arr[(best_idx + i) % 4].p;
        }
    }

    corners.assign(ordered.begin(), ordered.end());
}

void TraditionalBuffDetector::assign_slots(
    std::vector<DetectedTarget>& targets,
    const DetectedRCenter& r_center) const {

    constexpr double kSlotAngle = 2.0 * M_PI / static_cast<double>(NUM_SLOTS);

    for (auto& t : targets) {
        if (!t.valid) continue;

        if (r_center.valid) {
            order_corners(t.landmarks, t.center, r_center.center);

            // 数学坐标系 (x 右, y 上 → 像素 Y 取反)
            cv::Point2f vec = t.center - r_center.center;
            double angle = std::atan2(-vec.y, vec.x);
            t.angle = angle;

            int slot = static_cast<int>(std::round(angle / kSlotAngle)) % NUM_SLOTS;
            if (slot < 0) slot += NUM_SLOTS;
            t.slot_id = slot;

            // keypoint_count=0: 强制 RuneObserver 走传统 PnP 路径
        } else {
            t.slot_id = -1;
        }
    }
}

}  // namespace autobuff::detector
