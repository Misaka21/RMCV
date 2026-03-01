// 传统能量机关检测器实现 (2026)

#include "traditional_detector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <set>

#include <fmt/format.h>
#include <opencv2/imgproc.hpp>

#include <opencv2/highgui.hpp>

namespace autobuff::detector {

namespace {

constexpr double kPi = 3.14159265358979323846;

inline float norm2(const cv::Point2f& v) { return std::sqrt(v.x * v.x + v.y * v.y); }

inline cv::Point2f unit_vec(const cv::Point2f& v) {
    float n = norm2(v);
    if (n < 1e-6f) return {0.f, 0.f};
    return {v.x / n, v.y / n};
}

inline double angle_between_unit(const cv::Point2f& a_u, const cv::Point2f& b_u) {
    double dot = std::clamp(static_cast<double>(a_u.dot(b_u)), -1.0, 1.0);
    return std::acos(dot);
}

}  // namespace

TraditionalDetector::TraditionalDetector(const TraditionalDetectorConfig& config)
    : config_(config) {}

BuffDetectionResult TraditionalDetector::detect(const cv::Mat& image, double timestamp) {
    BuffDetectionResult result;
    result.timestamp = timestamp;
    result.enemy_color = enemy_color_;

    if (image.empty()) {
        result.update_summary();
        return result;
    }

    // 1) 色差二值化
    cv::Mat binary = binary_color_diff(image);

    // 2) 轮廓提取 (树结构)
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binary, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        result.update_summary();
        if (config_.show_window || config_.save_debug_image) {
            draw_debug_image(image, result);
        }
        return result;
    }

    // 3) 特征检测
    auto r_candidates = find_r_centers(contours, hierarchy);
    auto target_candidates = find_targets(contours, hierarchy);

    // 4) 选择最佳 R 标
    select_best_r_center(r_candidates, result.r_center);

    // 5) 分配槽位并输出 targets[0..4]
    assign_targets_to_slots(target_candidates, result.r_center, result.targets);

    // 6) 汇总 + 调试
    result.update_summary();
    if (config_.show_window || config_.save_debug_image) {
        draw_debug_image(image, result);
    }

    return result;
}

cv::Mat TraditionalDetector::binary_color_diff(const cv::Mat& src) const {
    cv::Mat out(src.size(), CV_8UC1);
    const int th = (enemy_color_ == EnemyColor::RED)
        ? config_.gray_threshold_red
        : config_.gray_threshold_blue;

    for (int y = 0; y < src.rows; ++y) {
        const auto* row = src.ptr<cv::Vec3b>(y);
        auto* dst = out.ptr<uint8_t>(y);
        for (int x = 0; x < src.cols; ++x) {
            // Project convention: input images are RGB (see auto_aim traditional pipeline).
            int r = row[x][0];
            int b = row[x][2];
            int diff = (enemy_color_ == EnemyColor::RED) ? (r - b) : (b - r);
            dst[x] = (diff > th) ? 255 : 0;
        }
    }

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(out, out, cv::MORPH_CLOSE, kernel);
    return out;
}

std::vector<TraditionalDetector::RCenterCandidate> TraditionalDetector::find_r_centers(
    const std::vector<std::vector<cv::Point>>& contours,
    const std::vector<cv::Vec4i>& hierarchy) const
{
    std::vector<RCenterCandidate> cands;

    for (int i = 0; i < static_cast<int>(contours.size()); ++i) {
        const auto& c = contours[i];
        double area = cv::contourArea(c);
        if (area < config_.r_area_min || area > config_.r_area_max) continue;

        // R 内部通常有孔洞 (子轮廓)
        if (hierarchy[i][2] < 0) continue;

        double peri = cv::arcLength(c, true);
        if (peri < 1e-3) continue;
        double circ = 4.0 * kPi * area / (peri * peri);
        if (circ < config_.r_circularity_min || circ > config_.r_circularity_max) continue;

        RCenterCandidate cand;
        cand.contour = c;
        cand.rect = cv::minAreaRect(c);
        cand.center = cand.rect.center;
        cand.area = area;
        cand.circularity = circ;
        cand.confidence = static_cast<float>(1.0 - std::abs(circ - 0.5) * 2.0);
        cands.push_back(std::move(cand));
    }

    return cands;
}

std::vector<TraditionalDetector::TargetCandidate> TraditionalDetector::find_targets(
    const std::vector<std::vector<cv::Point>>& contours,
    const std::vector<cv::Vec4i>& hierarchy) const
{
    std::vector<TargetCandidate> cands;

    for (int i = 0; i < static_cast<int>(contours.size()); ++i) {
        if (!is_top_level_with_child(hierarchy, i)) continue;

        const auto& c = contours[i];
        if (c.size() < 6) continue;

        double area = cv::contourArea(c);
        if (area < config_.target_area_min || area > config_.target_area_max) continue;

        cv::RotatedRect ellipse = cv::fitEllipse(c);
        double w = ellipse.size.width;
        double h = ellipse.size.height;
        if (w < 1e-3 || h < 1e-3) continue;
        double aspect = std::max(w, h) / std::min(w, h);
        if (aspect < config_.target_aspect_min || aspect > config_.target_aspect_max) continue;

        TargetCandidate cand;
        cand.outer_idx = i;
        cand.contour = c;
        cand.ellipse = ellipse;
        cand.center = ellipse.center;
        cand.area = area;
        cand.aspect_ratio = aspect;

        collect_all_sub_contours(hierarchy, i, cand.child_indices);

        // Lit(Active) 优先
        if (is_active_lit_target(contours, hierarchy, i, area)) {
            cand.is_lit = true;
            cand.confidence = static_cast<float>(1.0 / aspect);
            cands.push_back(std::move(cand));
            continue;
        }

        // Unlit(Inactive): 需要 gap+fan_tips
        if (is_inactive_unlit_target(contours, hierarchy, i, area)) {
            cand.is_lit = false;
            if (detect_gaps_and_fan_tips(cand, contours, hierarchy)) {
                cand.confidence = static_cast<float>(1.0 / aspect + 0.5);
                cands.push_back(std::move(cand));
            }
        }
    }

    return cands;
}

bool TraditionalDetector::select_best_r_center(
    const std::vector<RCenterCandidate>& cands,
    DetectedRCenter& out) const
{
    out.valid = false;
    if (cands.empty()) return false;

    const auto* best = &cands.front();
    for (const auto& c : cands) {
        if (c.confidence > best->confidence) best = &c;
    }

    out.center = best->center;
    out.confidence = best->confidence;
    out.valid = true;

    // 可选: 取 minAreaRect 的 4 角点作为 landmarks
    cv::Point2f vertices[4];
    best->rect.points(vertices);
    out.landmarks.assign(vertices, vertices + 4);
    return true;
}

void TraditionalDetector::assign_targets_to_slots(
    const std::vector<TargetCandidate>& cands,
    const DetectedRCenter& r_center,
    std::array<DetectedTarget, NUM_SLOTS>& out_targets) const
{
    for (auto& t : out_targets) t = DetectedTarget{};

    if (cands.empty()) return;
    if (!r_center.valid) {
        // 无中心时无法做 72deg 网格匹配: 仅输出置信度最高的前 N 个
        std::vector<const TargetCandidate*> sorted;
        sorted.reserve(cands.size());
        for (const auto& c : cands) sorted.push_back(&c);
        std::sort(sorted.begin(), sorted.end(),
                  [](const auto* a, const auto* b) { return a->confidence > b->confidence; });
        for (int i = 0; i < std::min<int>(NUM_SLOTS, static_cast<int>(sorted.size())); ++i) {
            const auto* c = sorted[i];
            out_targets[i].center = c->center;
            out_targets[i].slot_id = i;
            out_targets[i].is_lit = c->is_lit;
            out_targets[i].confidence = c->confidence;
            out_targets[i].valid = true;
        }
        return;
    }

    // 计算相对角度与距离
    struct AngleCand {
        const TargetCandidate* c = nullptr;
        double angle_rad = 0.0;  // math coord
        double angle_deg = 0.0;  // [0, 360)
        double dist = 0.0;
    };

    std::vector<AngleCand> angles;
    angles.reserve(cands.size());

    for (const auto& c : cands) {
        cv::Point2f d = c.center - r_center.center;
        double dist = std::hypot(d.x, d.y);
        if (dist < 30.0 || dist > 2000.0) continue;
        double a = std::atan2(-static_cast<double>(d.y), static_cast<double>(d.x));
        double a0 = normalize_angle_0_2pi(a);
        angles.push_back(AngleCand{&c, a, a0 * 180.0 / kPi, dist});
    }

    if (angles.empty()) return;

    // 距离一致性过滤 (用中位数更稳)
    std::vector<double> dists;
    dists.reserve(angles.size());
    for (const auto& ac : angles) dists.push_back(ac.dist);
    std::nth_element(dists.begin(), dists.begin() + dists.size() / 2, dists.end());
    double dist_med = dists[dists.size() / 2];
    angles.erase(std::remove_if(angles.begin(), angles.end(),
                    [&](const AngleCand& ac) {
                        return std::abs(ac.dist - dist_med) > dist_med * 0.35;
                    }),
                 angles.end());
    if (angles.empty()) return;

    // 只保留置信度最高的最多 5 个候选
    std::sort(angles.begin(), angles.end(),
              [](const AngleCand& a, const AngleCand& b) { return a.c->confidence > b.c->confidence; });
    if (static_cast<int>(angles.size()) > NUM_SLOTS) angles.resize(NUM_SLOTS);

    // 72deg 网格对齐: 求 rotate_deg
    std::vector<double> angle_deg_list;
    angle_deg_list.reserve(angles.size());
    for (const auto& ac : angles) angle_deg_list.push_back(ac.angle_deg);
    int rotate_deg = solve_best_rotate_deg(angle_deg_list);

    // 参考角 (deg)
    std::array<double, NUM_SLOTS> refs{};
    for (int i = 0; i < NUM_SLOTS; ++i) refs[i] = 72.0 * i - rotate_deg;

    // 排序后做唯一匹配
    std::sort(angles.begin(), angles.end(),
              [](const AngleCand& a, const AngleCand& b) { return a.angle_deg < b.angle_deg; });

    std::set<int> pending{0, 1, 2, 3, 4};
    for (const auto& ac : angles) {
        if (pending.empty()) break;
        int best_id = -1;
        double best_delta = 1e9;
        for (int id : pending) {
            double delta = std::abs(ac.angle_deg - refs[id]);
            if (delta < best_delta) {
                best_delta = delta;
                best_id = id;
            }
        }
        if (best_id < 0) continue;
        pending.erase(best_id);

        DetectedTarget out;
        out.center = ac.c->center;
        out.slot_id = best_id;
        out.angle = ac.angle_rad;
        out.is_lit = ac.c->is_lit;
        out.confidence = ac.c->confidence;
        out.valid = true;

        // 可选: 对 unlit 输出 fan tips landmarks
        if (!ac.c->is_lit && ac.c->has_fan_tips) {
            out.landmarks.clear();
            out.landmarks.reserve(5);
            out.landmarks.push_back(ac.c->center);
            // top/right/bottom/left
            out.landmarks.push_back(ac.c->fan_tips[0]);
            out.landmarks.push_back(ac.c->fan_tips[1]);
            out.landmarks.push_back(ac.c->fan_tips[2]);
            out.landmarks.push_back(ac.c->fan_tips[3]);
        }

        out_targets[best_id] = std::move(out);
    }
}

bool TraditionalDetector::is_top_level_with_child(const std::vector<cv::Vec4i>& hierarchy, int idx) {
    if (idx < 0) return false;
    if (hierarchy[idx][3] != -1) return false;  // parent exists
    if (hierarchy[idx][2] == -1) return false;  // no child
    return true;
}

void TraditionalDetector::collect_all_sub_contours(
    const std::vector<cv::Vec4i>& hierarchy,
    int idx,
    std::vector<int>& out)
{
    out.clear();
    if (idx < 0) return;
    std::vector<int> stack;
    int child = hierarchy[idx][2];
    while (child != -1) {
        out.push_back(child);
        stack.push_back(child);
        child = hierarchy[child][0];  // next sibling
    }

    // DFS: include all descendants
    while (!stack.empty()) {
        int cur = stack.back();
        stack.pop_back();
        int gc = hierarchy[cur][2];
        while (gc != -1) {
            out.push_back(gc);
            stack.push_back(gc);
            gc = hierarchy[gc][0];
        }
    }
}

bool TraditionalDetector::is_active_lit_target(
    const std::vector<std::vector<cv::Point>>& contours,
    const std::vector<cv::Vec4i>& hierarchy,
    int outer_idx,
    double outer_area) const
{
    // 必须顶层且有子轮廓
    if (!is_top_level_with_child(hierarchy, outer_idx)) return false;

    std::vector<int> all_sub;
    collect_all_sub_contours(hierarchy, outer_idx, all_sub);
    if (all_sub.empty()) return false;

    // 最大子轮廓面积比
    double max_sub_area = 0.0;
    int max_sub_idx = -1;
    double total_sub_area = 0.0;
    for (int si : all_sub) {
        double a = cv::contourArea(contours[si]);
        total_sub_area += a;
        if (a > max_sub_area) {
            max_sub_area = a;
            max_sub_idx = si;
        }
    }

    if (max_sub_idx < 0) return false;
    if (outer_area < 1e-3) return false;

    double ratio = max_sub_area / outer_area;
    if (ratio < config_.active_min_sub_area_ratio) {
        // 十环情况: 子轮廓很多但单个不大
        double total_ratio = total_sub_area / outer_area;
        return total_ratio <= config_.active_max_total_sub_area_ratio;
    }

    // 最大子轮廓也应近似椭圆
    const auto& sub = contours[max_sub_idx];
    if (sub.size() < 6) return false;
    cv::RotatedRect sub_ellipse = cv::fitEllipse(sub);
    double w = sub_ellipse.size.width;
    double h = sub_ellipse.size.height;
    if (w < 1e-3 || h < 1e-3) return false;
    double aspect = std::max(w, h) / std::min(w, h);
    if (aspect > 2.0) return false;

    // 同心性: 内外中心距离不能太大
    cv::RotatedRect outer_ellipse = cv::fitEllipse(contours[outer_idx]);
    double max_side = std::max(outer_ellipse.size.width, outer_ellipse.size.height);
    double center_dist = cv::norm(sub_ellipse.center - outer_ellipse.center);
    if (center_dist > 0.15 * max_side) return false;

    return true;
}

bool TraditionalDetector::is_inactive_unlit_target(
    const std::vector<std::vector<cv::Point>>& contours,
    const std::vector<cv::Vec4i>& hierarchy,
    int outer_idx,
    double /*outer_area*/) const
{
    // 必须顶层且有子轮廓 (gap)
    if (!is_top_level_with_child(hierarchy, outer_idx)) return false;

    // 统计直接子轮廓数量，至少 4
    int child = hierarchy[outer_idx][2];
    int child_count = 0;
    while (child != -1) {
        child_count++;
        child = hierarchy[child][0];
    }
    return child_count >= 4;
}

bool TraditionalDetector::detect_gaps_and_fan_tips(
    TargetCandidate& target,
    const std::vector<std::vector<cv::Point>>& contours,
    const std::vector<cv::Vec4i>& hierarchy) const
{
    target.gaps.clear();
    target.has_fan_tips = false;
    if (target.outer_idx < 0) return false;

    const int outer_idx = target.outer_idx;
    const double outer_area = target.area;
    const cv::RotatedRect outer_ellipse = target.ellipse;

    std::vector<Gap> gaps;
    gaps.reserve(4);

    for (int sub_idx : target.child_indices) {
        // 只取 outer 的直接 child 作为 gap 候选
        if (hierarchy[sub_idx][3] != outer_idx) continue;
        const auto& gc = contours[sub_idx];
        if (gc.size() < 10) continue;

        Gap g;
        if (make_gap(gc, outer_ellipse, outer_area, g)) {
            gaps.push_back(g);
        }
    }

    if (gaps.size() != 4) return false;

    // gap 排序: 以左上为起点顺时针/逆时针都行，这里使用数学坐标系做一致性
    auto gap_angle = [&](const Gap& g) -> double {
        cv::Point2f d = g.center - target.center;
        return std::atan2(-static_cast<double>(d.y), static_cast<double>(d.x));
    };

    std::sort(gaps.begin(), gaps.end(), [&](const Gap& a, const Gap& b) {
        auto normalize = [](double ang) {
            double adjusted = ang + kPi * 3.0 / 4.0;  // +135deg，使左上为起点
            if (adjusted < 0) adjusted += 2 * kPi;
            if (adjusted >= 2 * kPi) adjusted -= 2 * kPi;
            return adjusted;
        };
        return normalize(gap_angle(a)) < normalize(gap_angle(b));
    });

    target.gaps.assign(gaps.begin(), gaps.end());

    // 计算 fan tips: top/right/bottom/left
    for (int i = 0; i < 4; ++i) {
        int next = (i + 1) % 4;
        target.fan_tips[i] = (target.gaps[i].right_corner + target.gaps[next].left_corner) * 0.5f;
    }

    target.has_fan_tips = true;
    return true;
}

cv::Matx33f TraditionalDetector::get_ellipse_correction_mat(const cv::RotatedRect& ellipse, float& radius) {
    // 参考 rm_vision_core: 将外椭圆矫正为圆，便于 gap 角点稳定提取
    cv::Mat R2 = cv::getRotationMatrix2D(ellipse.center, ellipse.angle, 1.0);
    cv::Matx33f R(
        static_cast<float>(R2.at<double>(0, 0)), static_cast<float>(R2.at<double>(0, 1)), static_cast<float>(R2.at<double>(0, 2)),
        static_cast<float>(R2.at<double>(1, 0)), static_cast<float>(R2.at<double>(1, 1)), static_cast<float>(R2.at<double>(1, 2)),
        0.f, 0.f, 1.f
    );
    cv::Matx33f R_inv = R.inv();

    cv::Matx33f T(1.f, 0.f, -ellipse.center.x,
                  0.f, 1.f, -ellipse.center.y,
                  0.f, 0.f, 1.f);
    cv::Matx33f T_inv = T.inv();

    float len = static_cast<float>((ellipse.size.width + ellipse.size.height) * 2.0);
    radius = len * 0.5f;
    cv::Matx33f S(len / static_cast<float>(ellipse.size.width), 0.f, 0.f,
                  0.f, len / static_cast<float>(ellipse.size.height), 0.f,
                  0.f, 0.f, 1.f);
    return R_inv * T_inv * S * T * R;
}

bool TraditionalDetector::is_point_in_ellipse(const cv::Point2f& p, const cv::RotatedRect& ellipse) {
    // 未使用，保留以便后续扩展
    cv::Point2f tp = p - ellipse.center;
    float cos_a = std::cos(-ellipse.angle * static_cast<float>(kPi / 180.0));
    float sin_a = std::sin(-ellipse.angle * static_cast<float>(kPi / 180.0));
    cv::Point2f rp(tp.x * cos_a - tp.y * sin_a, tp.x * sin_a + tp.y * cos_a);
    float xr = ellipse.size.width * 0.5f;
    float yr = ellipse.size.height * 0.5f;
    if (xr < 1e-3f || yr < 1e-3f) return false;
    float nx = rp.x / xr;
    float ny = rp.y / yr;
    return nx * nx + ny * ny <= 1.0f;
}

std::pair<int, int> TraditionalDetector::get_left_right_idx(
    const std::vector<cv::Point2f>& contour,
    const cv::Point2f& center)
{
    int left_idx = 0;
    int right_idx = 0;
    cv::Point2f left_v = contour[0] - center;
    cv::Point2f right_v = contour[0] - center;
    for (int i = 0; i < static_cast<int>(contour.size()); ++i) {
        cv::Point2f v = contour[i] - center;
        if (left_v.cross(v) < 0) {
            left_v = v;
            left_idx = i;
        }
        if (right_v.cross(v) > 0) {
            right_v = v;
            right_idx = i;
        }
    }
    return {left_idx, right_idx};
}

bool TraditionalDetector::make_gap(
    const std::vector<cv::Point>& gap_contour,
    const cv::RotatedRect& outer_ellipse,
    double outer_area,
    Gap& out_gap) const
{
    out_gap.valid = false;

    double gap_area = cv::contourArea(gap_contour);
    if (outer_area < 1e-3) return false;
    double area_ratio = gap_area / outer_area;
    if (area_ratio < config_.gap_area_ratio_min || area_ratio > config_.gap_area_ratio_max) return false;

    if (gap_contour.size() < 10) return false;

    cv::RotatedRect gap_rect = cv::minAreaRect(gap_contour);
    double w = gap_rect.size.width;
    double h = gap_rect.size.height;
    if (w < 1e-3 || h < 1e-3) return false;
    double aspect = std::max(w, h) / std::min(w, h);
    if (aspect < config_.gap_aspect_min || aspect > config_.gap_aspect_max) return false;

    // 椭圆矫正
    float correction_radius = 0.f;
    cv::Matx33f M = get_ellipse_correction_mat(outer_ellipse, correction_radius);
    if (correction_radius < 1e-3f) return false;

    std::vector<cv::Point2f> corr;
    corr.reserve(gap_contour.size());
    for (const auto& p : gap_contour) {
        cv::Vec3f v = M * cv::Vec3f(static_cast<float>(p.x), static_cast<float>(p.y), 1.f);
        corr.emplace_back(v[0], v[1]);
    }

    if (corr.size() < 6) return false;
    cv::RotatedRect corr_ellipse = cv::fitEllipse(corr);

    // gap 中心在圆环上 (距离比)
    double center_dist = cv::norm(corr_ellipse.center - outer_ellipse.center);
    double ratio = center_dist / correction_radius;
    if (ratio < config_.gap_min_center_dist_ratio || ratio > config_.gap_max_center_dist_ratio) return false;

    // 左右角点
    auto [li, ri] = get_left_right_idx(corr, outer_ellipse.center);
    cv::Point2f left_u = unit_vec(corr[li] - outer_ellipse.center);
    cv::Point2f right_u = unit_vec(corr[ri] - outer_ellipse.center);
    cv::Point2f center_u = unit_vec(corr_ellipse.center - outer_ellipse.center);

    float r_circle = static_cast<float>(config_.gap_circle_radius_ratio) * correction_radius;
    cv::Point2f left_c = left_u * r_circle + outer_ellipse.center;
    cv::Point2f right_c = right_u * r_circle + outer_ellipse.center;
    cv::Point2f gap_c = center_u * r_circle + outer_ellipse.center;

    // 角度约束: 左右角点夹角约 72deg~90deg (经验范围)
    double delta = angle_between_unit(unit_vec(left_c - outer_ellipse.center),
                                      unit_vec(right_c - outer_ellipse.center));
    double delta_deg = delta * 180.0 / kPi;
    if (delta_deg < 60.0 || delta_deg > 110.0) return false;

    // gap 方向约束: 缺口长轴与径向近似垂直
    cv::Point2f gap_dir = unit_vec(right_c - left_c);
    cv::Point2f radial = unit_vec(gap_c - outer_ellipse.center);
    double dir_deg = angle_between_unit(gap_dir, radial) * 180.0 / kPi;
    if (dir_deg < 70.0 || dir_deg > 110.0) return false;

    // 反变换回原图
    cv::Matx33f M_inv = M.inv();
    auto inv_point = [&](const cv::Point2f& p) -> cv::Point2f {
        cv::Vec3f v = M_inv * cv::Vec3f(p.x, p.y, 1.f);
        return {v[0], v[1]};
    };

    out_gap.left_corner = inv_point(left_c);
    out_gap.right_corner = inv_point(right_c);
    out_gap.center = inv_point(gap_c);
    out_gap.valid = true;
    return true;
}

double TraditionalDetector::normalize_angle_0_2pi(double a) {
    while (a < 0) a += 2 * kPi;
    while (a >= 2 * kPi) a -= 2 * kPi;
    return a;
}

int TraditionalDetector::solve_best_rotate_deg(const std::vector<double>& angles_deg) {
    if (angles_deg.empty()) return 0;
    constexpr std::array<double, NUM_SLOTS> refs{0.0, 72.0, 144.0, 216.0, 288.0};

    int best_rot = 0;
    double best_cost = std::numeric_limits<double>::infinity();

    for (int rot = -180; rot < 180; ++rot) {
        double cost = 0.0;
        for (double a : angles_deg) {
            double ar = a + rot;
            double min_d = 1e9;
            for (double r : refs) {
                min_d = std::min(min_d, std::abs(ar - r));
            }
            cost += min_d * min_d;
        }
        if (cost < best_cost) {
            best_cost = cost;
            best_rot = rot;
        }
    }
    return best_rot;
}

void TraditionalDetector::draw_debug_image(const cv::Mat& src, const BuffDetectionResult& result) {
    debug_image_ = src.clone();

    const cv::Scalar r_color(0, 255, 255);
    const cv::Scalar unlit_color(0, 255, 0);
    const cv::Scalar lit_color(0, 0, 255);
    const cv::Scalar tip_color(255, 255, 0);

    if (result.r_center.valid) {
        cv::circle(debug_image_, result.r_center.center, 20, r_color, 2);
        cv::putText(debug_image_, "R", result.r_center.center + cv::Point2f(-10, 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, r_color, 2);
    }

    for (int i = 0; i < NUM_SLOTS; ++i) {
        const auto& t = result.targets[i];
        if (!t.valid) continue;

        cv::Scalar c = t.is_lit ? lit_color : unlit_color;
        cv::circle(debug_image_, t.center, 15, c, 2);
        cv::putText(debug_image_, fmt::format("{}", i),
                    t.center + cv::Point2f(18, 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, c, 2);

        if (t.landmarks.size() >= 5) {
            for (int k = 1; k < 5; ++k) {
                cv::circle(debug_image_, t.landmarks[k], 4, tip_color, -1);
            }
        }
    }

    std::string status_str;
    switch (result.status) {
        case DetectionStatus::NONE: status_str = "NONE"; break;
        case DetectionStatus::R_ONLY: status_str = "R_ONLY"; break;
        case DetectionStatus::TARGETS_ONLY: status_str = "TARGETS_ONLY"; break;
        case DetectionStatus::PARTIAL: status_str = "PARTIAL"; break;
        case DetectionStatus::COMPLETE: status_str = "COMPLETE"; break;
    }
    cv::putText(debug_image_, "Status: " + status_str,
                cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                cv::Scalar(255, 255, 255), 2);

    cv::putText(debug_image_, fmt::format("Lit: {}", result.lit_count),
                cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                lit_color, 2);

    if (config_.show_window) {
        cv::imshow("Buff Detector", debug_image_);
        cv::waitKey(1);
    }
}

}  // namespace autobuff::detector
