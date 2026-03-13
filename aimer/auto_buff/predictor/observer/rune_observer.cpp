// RuneObserver 实现: PnP 几何解算

#include "rune_observer.hpp"

#include <cmath>
#include <vector>

#include <opencv2/calib3d.hpp>

#include "aimer/common/transformer/transformer.hpp"

namespace autobuff::predictor {

namespace {

// YOLOX 装甲板角点 3D 物理坐标 (能量机关坐标系, 单位: m)
//
// FYT yolox_rune_3.6m 模型输出 4 个装甲板角点:
//   kpt[0] = bottom_left  (靠近R标, 左侧, 半径 541.5mm, 水平偏移 186mm)
//   kpt[1] = top_left     (远离R标, 左侧, 半径 858.5mm, 水平偏移 160mm)
//   kpt[2] = top_right    (远离R标, 右侧, 半径 858.5mm, 水平偏移 160mm)
//   kpt[3] = bottom_right (靠近R标, 右侧, 半径 541.5mm, 水平偏移 186mm)
//
// 坐标约定 (base, slot_id=0):
//   x = 0 (法向, 始终为 0)
//   y = 水平偏移 (+y = 右侧)
//   z = 径向距离 (+z = 远离 R 标)
//
// 对于其他 slot, 需要绕原点旋转 slot_id * 72°
const std::array<cv::Point3f, 4> ARMOR_OBJECT_POINTS_BASE = {{
    {0.f, -0.186f, 0.5415f},   // kpt[0] bottom_left  (内侧, 左)
    {0.f, -0.160f, 0.8585f},   // kpt[1] top_left     (外侧, 左)
    {0.f,  0.160f, 0.8585f},   // kpt[2] top_right    (外侧, 右)
    {0.f,  0.186f, 0.5415f},   // kpt[3] bottom_right (内侧, 右)
}};

// 将 base keypoint 绕原点旋转 theta (在 YZ 平面的 2D 旋转)
// pt.y, pt.z 构成扇叶平面内的坐标, pt.x = 0 (法向偏移)
inline cv::Point3f rotate_blade_point(const cv::Point3f& pt, double theta) {
    float cos_t = static_cast<float>(std::cos(theta));
    float sin_t = static_cast<float>(std::sin(theta));
    return {
        pt.z * cos_t - pt.y * sin_t,  // X: 径向投影
        pt.z * sin_t + pt.y * cos_t,  // Y: 径向投影
        pt.x                           // Z: 法向 (=0)
    };
}

}  // namespace

RuneObservation RuneObserver::observe(const BuffDetectionResult& det) const {
    RuneObservation obs;

    // 收集 PnP 点对
    std::vector<cv::Point2f> img_pts;
    std::vector<cv::Point3f> obj_pts;
    img_pts.reserve(4 * NUM_SLOTS + 1);
    obj_pts.reserve(4 * NUM_SLOTS + 1);

    // R 中心作为原点 (可选)
    if (det.r_center.valid) {
        img_pts.push_back(det.r_center.center);
        obj_pts.emplace_back(0.f, 0.f, 0.f);
    }

    for (int i = 0; i < NUM_SLOTS; ++i) {
        if (!det.targets[i].valid) continue;

        double theta = i * 2.0 * M_PI / NUM_SLOTS;

        if (det.targets[i].keypoint_count >= 4) {
            // YOLOX 路径: 每个扇叶贡献 4 个装甲板角点
            for (int k = 0; k < 4; ++k) {
                img_pts.push_back(det.targets[i].keypoints[k]);
                obj_pts.push_back(
                    rotate_blade_point(ARMOR_OBJECT_POINTS_BASE[k], theta));
            }
        } else {
            // 传统路径: 仅用扇叶中心
            img_pts.push_back(det.targets[i].center);
            obj_pts.emplace_back(
                static_cast<float>(autobuff::RUNE_RADIUS * std::cos(theta)),
                static_cast<float>(autobuff::RUNE_RADIUS * std::sin(theta)),
                0.f
            );
        }
    }

    if (img_pts.size() < 4) {
        obs.valid = false;
        return obs;
    }

    // solvePnP
    const cv::Mat& K = aimer::tf::get_camera_matrix();
    const cv::Mat& D = aimer::tf::get_distort_coeffs();

    cv::Mat rvec, tvec;
    bool ok = cv::solvePnP(obj_pts, img_pts, K, D, rvec, tvec, false,
                           cv::SOLVEPNP_ITERATIVE);
    if (!ok) {
        obs.valid = false;
        return obs;
    }

    cv::Mat Rcv;
    cv::Rodrigues(rvec, Rcv);
    Eigen::Matrix3d R;
    R << Rcv.at<double>(0, 0), Rcv.at<double>(0, 1), Rcv.at<double>(0, 2),
         Rcv.at<double>(1, 0), Rcv.at<double>(1, 1), Rcv.at<double>(1, 2),
         Rcv.at<double>(2, 0), Rcv.at<double>(2, 1), Rcv.at<double>(2, 2);

    Eigen::Vector3d t(
        tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

    obs.center_cam = t;
    obs.center_world = aimer::tf::cam_to_world(t, det.robot_state.q_imu);

    // 轮盘法向量 (相机坐标系 +Z 方向朝向相机)
    Eigen::Vector3d n_cam = R.col(2);
    if (n_cam.z() < 0) n_cam = -n_cam;
    obs.normal_cam = n_cam.normalized();

    // 各槽位 3D 信息
    for (int i = 0; i < NUM_SLOTS; ++i) {
        if (!det.targets[i].valid) continue;
        double theta = i * 2.0 * M_PI / NUM_SLOTS;
        Eigen::Vector3d p_obj(
            autobuff::RUNE_RADIUS * std::cos(theta),
            autobuff::RUNE_RADIUS * std::sin(theta),
            0.0
        );
        Eigen::Vector3d p_cam = R * p_obj + t;
        obs.slots[i].valid = true;
        obs.slots[i].pos_cam = p_cam;
        obs.slots[i].pos_world = aimer::tf::cam_to_world(p_cam, det.robot_state.q_imu);
        obs.slots[i].vec_cam = p_cam - t;
    }

    obs.valid = true;
    return obs;
}

}  // namespace autobuff::predictor
