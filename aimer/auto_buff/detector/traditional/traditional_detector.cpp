//
// 传统能量机关检测器实现
//

#include "traditional_detector.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace autobuff::detector {

// ============================================================================
// 构造函数
// ============================================================================

TraditionalDetector::TraditionalDetector(const TraditionalDetectorConfig& config)
    : config_(config) {}

// ============================================================================
// 公共接口
// ============================================================================

BuffDetectionResult TraditionalDetector::detect(const cv::Mat& image, double timestamp) {
    BuffDetectionResult result;
    result.timestamp = timestamp;
    result.frame_id = frame_count_++;
    result.enemy_color = enemy_color_;

    if (image.empty()) {
        return result;
    }

    // 1. 色差二值化
    cv::Mat binary = binary_color_diff(image);

    // 2. 轮廓提取
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binary, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        result.update_status();
        if (config_.show_window || config_.save_debug_image) {
            draw_debug_image(image, result);
        }
        return result;
    }

    // 3. 特征检测
    auto r_candidates = find_r_centers(binary, contours, hierarchy);
    auto target_candidates = find_targets(binary, contours, hierarchy);
    auto arrow_candidates = find_arrows(binary, contours);

    // 4. 选择最佳R标
    select_best_r_center(r_candidates, result.r_center);

    // 5. 过滤并关联靶心
    filter_and_match_targets(target_candidates, result.r_center, result.targets);

    // 6. 选择箭头并确定激活目标
    select_best_arrow(arrow_candidates, result.r_center, result.targets,
                      result.arrow, result.active_slot_id);

    // 7. 更新状态
    result.update_status();

    // 8. 绘制调试图像
    if (config_.show_window || config_.save_debug_image) {
        draw_debug_image(image, result);
    }

    return result;
}

void TraditionalDetector::set_enemy_color(EnemyColor color) {
    enemy_color_ = color;
}

EnemyColor TraditionalDetector::get_enemy_color() const {
    return enemy_color_;
}

cv::Mat TraditionalDetector::get_debug_image() const {
    return debug_image_;
}

// ============================================================================
// 色差二值化
// ============================================================================

cv::Mat TraditionalDetector::binary_color_diff(const cv::Mat& src) {
    cv::Mat result(src.size(), CV_8UC1);

    // 直接使用配置中的阈值
    int threshold = (enemy_color_ == EnemyColor::RED)
        ? config_.gray_threshold_red
        : config_.gray_threshold_blue;

    for (int y = 0; y < src.rows; ++y) {
        const auto* src_row = src.ptr<cv::Vec3b>(y);
        auto* dst_row = result.ptr<uint8_t>(y);

        for (int x = 0; x < src.cols; ++x) {
            int b = src_row[x][0];
            int g = src_row[x][1];
            int r = src_row[x][2];

            // 色差计算
            int diff = (enemy_color_ == EnemyColor::RED) ? (r - b) : (b - r);
            dst_row[x] = (diff > threshold) ? 255 : 0;
        }
    }

    // 形态学处理
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(result, result, cv::MORPH_CLOSE, kernel);

    return result;
}

// ============================================================================
// R标检测
// ============================================================================

std::vector<TraditionalDetector::RCenterCandidate> TraditionalDetector::find_r_centers(
    const cv::Mat& binary,
    const std::vector<std::vector<cv::Point>>& contours,
    const std::vector<cv::Vec4i>& hierarchy) {

    std::vector<RCenterCandidate> candidates;

    for (size_t i = 0; i < contours.size(); ++i) {
        const auto& contour = contours[i];
        double area = cv::contourArea(contour);

        // 面积过滤
        if (area < config_.r_area_min || area > config_.r_area_max) {
            continue;
        }

        // 必须有子轮廓 (R字内部)
        if (hierarchy[i][2] < 0) {
            continue;
        }

        // 计算圆度
        double perimeter = cv::arcLength(contour, true);
        double circularity = 4 * M_PI * area / (perimeter * perimeter);

        if (circularity < config_.r_circularity_min ||
            circularity > config_.r_circularity_max) {
            continue;
        }

        RCenterCandidate candidate;
        candidate.contour = contour;
        candidate.rect = cv::minAreaRect(contour);
        candidate.center = candidate.rect.center;
        candidate.area = area;
        candidate.circularity = circularity;

        // 置信度: 圆度越接近0.5越好 (R标既不是完美圆也不是方形)
        candidate.confidence = 1.0f - std::abs(circularity - 0.5f) * 2.0f;

        candidates.push_back(candidate);
    }

    return candidates;
}

// ============================================================================
// 靶心检测
// ============================================================================

std::vector<TraditionalDetector::TargetCandidate> TraditionalDetector::find_targets(
    const cv::Mat& binary,
    const std::vector<std::vector<cv::Point>>& contours,
    const std::vector<cv::Vec4i>& hierarchy) {

    std::vector<TargetCandidate> candidates;

    for (size_t i = 0; i < contours.size(); ++i) {
        const auto& contour = contours[i];
        double area = cv::contourArea(contour);

        // 面积过滤
        if (area < config_.target_area_min || area > config_.target_area_max) {
            continue;
        }

        cv::RotatedRect rect = cv::minAreaRect(contour);
        double w = rect.size.width;
        double h = rect.size.height;
        double aspect = std::max(w, h) / std::min(w, h);

        // 长宽比过滤 (靶心接近正方形)
        if (aspect < config_.target_aspect_min || aspect > config_.target_aspect_max) {
            continue;
        }

        TargetCandidate candidate;
        candidate.contour = contour;
        candidate.rect = rect;
        candidate.center = rect.center;
        candidate.area = area;
        candidate.aspect_ratio = aspect;

        // 检查子轮廓判断激活状态
        // 未激活: 有4个以上子轮廓 (4个缺口)
        // 已激活: 子轮廓呈同心圆结构
        int child_count = 0;
        int child_idx = hierarchy[i][2];
        while (child_idx >= 0) {
            candidate.inner_contours.push_back(contours[child_idx]);
            child_count++;
            child_idx = hierarchy[child_idx][0];
        }

        // 简单判断: 子轮廓数>=4为未激活
        candidate.is_active = (child_count < 4);

        // 置信度: 长宽比越接近1越好
        candidate.confidence = 1.0f / aspect;

        candidates.push_back(candidate);
    }

    return candidates;
}

// ============================================================================
// 箭头检测
// ============================================================================

std::vector<TraditionalDetector::ArrowCandidate> TraditionalDetector::find_arrows(
    const cv::Mat& binary,
    const std::vector<std::vector<cv::Point>>& contours) {

    std::vector<ArrowCandidate> candidates;

    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);

        // 面积过滤
        if (area < config_.arrow_area_min || area > config_.arrow_area_max) {
            continue;
        }

        cv::RotatedRect rect = cv::minAreaRect(contour);
        double w = rect.size.width;
        double h = rect.size.height;
        double aspect = std::max(w, h) / std::min(w, h);

        // 长宽比过滤 (箭头是长条形)
        if (aspect < config_.arrow_aspect_min || aspect > config_.arrow_aspect_max) {
            continue;
        }

        ArrowCandidate candidate;
        candidate.contour = contour;
        candidate.rect = rect;
        candidate.center = rect.center;
        candidate.area = area;
        candidate.aspect_ratio = aspect;

        // 计算箭头方向: 使用旋转矩形的角度
        double angle = rect.angle;
        if (w < h) {
            angle += 90;
        }
        candidate.angle = angle * M_PI / 180.0;

        // 确定 tip 和 tail
        // 沿长轴方向找两端点
        cv::Point2f vertices[4];
        rect.points(vertices);

        // 按长轴排序
        std::vector<cv::Point2f> pts(vertices, vertices + 4);
        cv::Point2f axis_dir(std::cos(candidate.angle), std::sin(candidate.angle));

        // 找沿轴最远的两个点
        auto proj = [&](const cv::Point2f& p) {
            return p.dot(axis_dir);
        };

        std::sort(pts.begin(), pts.end(), [&](const cv::Point2f& a, const cv::Point2f& b) {
            return proj(a) < proj(b);
        });

        candidate.tail = (pts[0] + pts[1]) * 0.5f;
        candidate.tip = (pts[2] + pts[3]) * 0.5f;

        // 置信度: 长宽比越大越好
        candidate.confidence = std::min(1.0f, static_cast<float>(aspect / config_.arrow_aspect_max));

        candidates.push_back(candidate);
    }

    return candidates;
}

// ============================================================================
// 特征选择与关联
// ============================================================================

bool TraditionalDetector::select_best_r_center(
    const std::vector<RCenterCandidate>& candidates,
    DetectedRCenter& result) {

    if (candidates.empty()) {
        result.valid = false;
        return false;
    }

    // 选择置信度最高的
    const auto* best = &candidates[0];
    for (const auto& c : candidates) {
        if (c.confidence > best->confidence) {
            best = &c;
        }
    }

    result.center = best->center;
    result.confidence = best->confidence;
    result.valid = true;

    // 如果轮廓点数>=4，存储为landmarks
    if (best->contour.size() >= 4) {
        // 简化: 取旋转矩形4角点
        cv::Point2f vertices[4];
        best->rect.points(vertices);
        result.landmarks.assign(vertices, vertices + 4);
    }

    return true;
}

void TraditionalDetector::filter_and_match_targets(
    const std::vector<TargetCandidate>& candidates,
    const DetectedRCenter& r_center,
    std::array<DetectedTarget, NUM_SLOTS>& targets) {

    // 重置所有靶心
    for (auto& t : targets) {
        t.valid = false;
    }

    if (candidates.empty()) {
        return;
    }

    // 如果没有R标，无法进行几何约束
    // 直接按置信度排序取前NUM_SLOTS个
    if (!r_center.valid) {
        std::vector<const TargetCandidate*> sorted;
        for (const auto& c : candidates) {
            sorted.push_back(&c);
        }
        std::sort(sorted.begin(), sorted.end(),
            [](const TargetCandidate* a, const TargetCandidate* b) {
                return a->confidence > b->confidence;
            });

        for (size_t i = 0; i < std::min(sorted.size(), static_cast<size_t>(NUM_SLOTS)); ++i) {
            const auto* c = sorted[i];
            targets[i].center = c->center;
            targets[i].slot_id = static_cast<int>(i);
            targets[i].is_active = c->is_active;
            targets[i].confidence = c->confidence;
            targets[i].valid = true;
        }
        return;
    }

    // 有R标时，按角度分配槽位
    // 计算每个靶心相对R标的角度
    struct AngleTarget {
        const TargetCandidate* candidate;
        double angle;  // 相对R标的角度 (rad)
        double distance;
    };

    std::vector<AngleTarget> angle_targets;
    for (const auto& c : candidates) {
        cv::Point2f diff = c.center - r_center.center;
        double distance = cv::norm(diff);

        // 距离过滤: 应该在合理范围内
        // 能量机关半径约700mm，图像中约100-500像素
        if (distance < 50 || distance > 800) {
            continue;
        }

        double angle = std::atan2(diff.y, diff.x);  // [-PI, PI]
        angle_targets.push_back({&c, angle, distance});
    }

    if (angle_targets.empty()) {
        return;
    }

    // 计算平均距离
    double avg_distance = 0;
    for (const auto& at : angle_targets) {
        avg_distance += at.distance;
    }
    avg_distance /= angle_targets.size();

    // 分配槽位: 5个槽位均匀分布在圆周上
    // 槽位0在正上方 (angle = -PI/2)
    constexpr double SLOT_ANGLE_STEP = 2 * M_PI / NUM_SLOTS;
    constexpr double SLOT_0_ANGLE = -M_PI / 2;

    for (const auto& at : angle_targets) {
        // 距离过滤
        if (std::abs(at.distance - avg_distance) > avg_distance * 0.3) {
            continue;
        }

        // 计算最近的槽位
        double normalized_angle = at.angle - SLOT_0_ANGLE;
        while (normalized_angle < 0) normalized_angle += 2 * M_PI;
        while (normalized_angle >= 2 * M_PI) normalized_angle -= 2 * M_PI;

        int slot_id = static_cast<int>(std::round(normalized_angle / SLOT_ANGLE_STEP)) % NUM_SLOTS;

        // 检查槽位是否已占用，如果新候选置信度更高则替换
        if (!targets[slot_id].valid ||
            at.candidate->confidence > targets[slot_id].confidence) {

            targets[slot_id].center = at.candidate->center;
            targets[slot_id].slot_id = slot_id;
            targets[slot_id].angle_from_center = at.angle;
            targets[slot_id].is_active = at.candidate->is_active;
            targets[slot_id].confidence = at.candidate->confidence;
            targets[slot_id].valid = true;

            // 存储landmarks (如果有内轮廓，提取缺口位置)
            // 简化: 取旋转矩形4角点 + 中心
            cv::Point2f vertices[4];
            at.candidate->rect.points(vertices);
            targets[slot_id].landmarks.clear();
            targets[slot_id].landmarks.assign(vertices, vertices + 4);
            targets[slot_id].landmarks.push_back(at.candidate->center);
        }
    }
}

bool TraditionalDetector::select_best_arrow(
    const std::vector<ArrowCandidate>& candidates,
    const DetectedRCenter& r_center,
    const std::array<DetectedTarget, NUM_SLOTS>& targets,
    DetectedArrow& result,
    int& active_slot_id) {

    result.valid = false;
    active_slot_id = -1;

    if (candidates.empty()) {
        return false;
    }

    // 如果没有R标或靶心，无法关联
    if (!r_center.valid) {
        // 简单选择置信度最高的箭头
        const auto* best = &candidates[0];
        for (const auto& c : candidates) {
            if (c.confidence > best->confidence) {
                best = &c;
            }
        }
        result.tip = best->tip;
        result.tail = best->tail;
        result.direction_angle = best->angle;
        result.confidence = best->confidence;
        result.valid = true;
        return true;
    }

    // 有R标时，选择指向R标的箭头
    const ArrowCandidate* best = nullptr;
    float best_score = 0;
    int best_slot = -1;

    for (const auto& c : candidates) {
        // 检查箭头是否指向R标
        // tip应该比tail更接近R标
        double tip_dist = cv::norm(c.tip - r_center.center);
        double tail_dist = cv::norm(c.tail - r_center.center);

        if (tip_dist >= tail_dist) {
            continue;  // 方向错误
        }

        // 找箭头尾部最近的靶心
        int nearest_slot = -1;
        double min_dist = std::numeric_limits<double>::max();

        for (int i = 0; i < NUM_SLOTS; ++i) {
            if (!targets[i].valid || targets[i].is_active) {
                continue;  // 只匹配未激活的靶心
            }

            double dist = cv::norm(c.tail - targets[i].center);
            if (dist < min_dist) {
                min_dist = dist;
                nearest_slot = i;
            }
        }

        if (nearest_slot < 0) {
            continue;
        }

        // 评分: 置信度 + 距离因子
        float score = c.confidence - static_cast<float>(min_dist * 0.001);

        if (score > best_score) {
            best_score = score;
            best = &c;
            best_slot = nearest_slot;
        }
    }

    if (best == nullptr) {
        return false;
    }

    result.tip = best->tip;
    result.tail = best->tail;
    result.direction_angle = best->angle;
    result.target_slot_id = best_slot;
    result.confidence = best->confidence;
    result.valid = true;

    active_slot_id = best_slot;
    return true;
}

// ============================================================================
// 准确度计算
// ============================================================================

double TraditionalDetector::compute_accuracy(
    const DetectedRCenter& r_center,
    const std::array<DetectedTarget, NUM_SLOTS>& targets) {

    if (!r_center.valid) {
        return 0.0;
    }

    int valid_count = 0;
    double distance_ratio_sum = 0;

    std::vector<double> distances;
    for (const auto& t : targets) {
        if (t.valid) {
            double dist = cv::norm(t.center - r_center.center);
            distances.push_back(dist);
            valid_count++;
        }
    }

    if (valid_count < 2) {
        return 0.5;  // 信息不足
    }

    // 计算距离比率的一致性
    double avg_dist = 0;
    for (double d : distances) {
        avg_dist += d;
    }
    avg_dist /= distances.size();

    for (double d : distances) {
        double ratio = std::min(d, avg_dist) / std::max(d, avg_dist);
        distance_ratio_sum += ratio;
    }

    return distance_ratio_sum / distances.size();
}

// ============================================================================
// 调试绘制
// ============================================================================

void TraditionalDetector::draw_debug_image(const cv::Mat& src,
                                            const BuffDetectionResult& result) {
    debug_image_ = src.clone();

    // 颜色定义
    cv::Scalar r_color(0, 255, 255);       // 黄色: R标
    cv::Scalar target_color(0, 255, 0);    // 绿色: 靶心
    cv::Scalar active_color(0, 0, 255);    // 红色: 待打击目标
    cv::Scalar arrow_color(255, 0, 255);   // 紫色: 箭头

    // 绘制R标
    if (result.r_center.valid) {
        cv::circle(debug_image_, result.r_center.center, 20, r_color, 2);
        cv::putText(debug_image_, "R",
                    result.r_center.center + cv::Point2f(-10, 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, r_color, 2);
    }

    // 绘制靶心
    for (int i = 0; i < NUM_SLOTS; ++i) {
        const auto& t = result.targets[i];
        if (!t.valid) continue;

        cv::Scalar color = (i == result.active_slot_id) ? active_color : target_color;
        int thickness = (i == result.active_slot_id) ? 3 : 2;

        cv::circle(debug_image_, t.center, 15, color, thickness);
        cv::putText(debug_image_,
                    std::to_string(i) + (t.is_active ? "*" : ""),
                    t.center + cv::Point2f(20, 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
    }

    // 绘制箭头
    if (result.arrow.valid) {
        cv::arrowedLine(debug_image_, result.arrow.tail, result.arrow.tip,
                        arrow_color, 2, cv::LINE_AA, 0, 0.3);
    }

    // 绘制状态文本
    std::string status_str;
    switch (result.status) {
        case DetectionStatus::NONE: status_str = "NONE"; break;
        case DetectionStatus::R_ONLY: status_str = "R_ONLY"; break;
        case DetectionStatus::TARGET_ONLY: status_str = "TARGET_ONLY"; break;
        case DetectionStatus::PARTIAL: status_str = "PARTIAL"; break;
        case DetectionStatus::COMPLETE: status_str = "COMPLETE"; break;
    }

    cv::putText(debug_image_, "Status: " + status_str,
                cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                cv::Scalar(255, 255, 255), 2);

    if (result.active_slot_id >= 0) {
        cv::putText(debug_image_, "Target: " + std::to_string(result.active_slot_id),
                    cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                    active_color, 2);
    }

    // 显示窗口
    if (config_.show_window) {
        cv::imshow("Buff Detector", debug_image_);
        cv::waitKey(1);
    }
}

}  // namespace autobuff::detector
