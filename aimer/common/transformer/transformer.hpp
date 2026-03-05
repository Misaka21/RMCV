//
// Created by nuc11 on 2025/10/5.
// TF树坐标变换系统 - 编译期自动推导变换链
//

#ifndef RMCV_TRANSFORMER_HPP
#define RMCV_TRANSFORMER_HPP

#include <string>
#include <type_traits>

#include <Eigen/Dense>
#include <opencv2/core/mat.hpp>

#include "aimer/common/math/math.hpp"

namespace aimer::tf {

using aimer::math::YpdCoord;

// ============================================================================
// 1. 坐标系定义
// ============================================================================

enum class Frame {
    World,   // 大地坐标系 (预测在此进行)
    Imu,     // IMU芯片坐标系 (可能侧着装)
    Gimbal,  // 云台坐标系 (修正IMU安装偏差后)
    Camera,  // 相机坐标系 (PnP结果)
    Barrel   // 枪口坐标系 (弹道起点)
};

// ============================================================================
// 2. TF树结构定义 (编译期)
//
//   World (大地坐标系)
//     │
//     └── Imu (IMU芯片坐标系，q_imu)
//          │
//          └── Gimbal (云台坐标系，R_gimbal2imubody修正)
//               │
//               ├── Camera (相机外参)
//               │
//               └── Barrel (枪口偏移)
//
// ============================================================================

// 父节点关系
template<Frame F> struct Parent;
template<> struct Parent<Frame::World>  { static constexpr Frame value = Frame::World; };
template<> struct Parent<Frame::Imu>    { static constexpr Frame value = Frame::World; };
template<> struct Parent<Frame::Gimbal> { static constexpr Frame value = Frame::Imu; };
template<> struct Parent<Frame::Camera> { static constexpr Frame value = Frame::Gimbal; };
template<> struct Parent<Frame::Barrel> { static constexpr Frame value = Frame::Gimbal; };

// ============================================================================
// 3. 单步变换定义
// ============================================================================

// 变换基类模板
template<Frame Child, Frame Par = Parent<Child>::value>
struct Transform {
    static Eigen::Matrix4d get(const Eigen::Quaterniond&) {
        return Eigen::Matrix4d::Identity();
    }
};

// Imu → World (动态变换，依赖IMU四元数)
template<>
struct Transform<Frame::Imu, Frame::World> {
    static Eigen::Matrix4d get(const Eigen::Quaterniond& q_imu) {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = q_imu.toRotationMatrix();
        return T;
    }
};

// Gimbal → Imu (静态变换，修正IMU安装偏差)
// R_gimbal2imubody: 从Gimbal到Imu的旋转
// 我们需要的是 Gimbal→Imu，即 R_gimbal2imubody
template<>
struct Transform<Frame::Gimbal, Frame::Imu> {
    inline static Eigen::Matrix3d R_ = Eigen::Matrix3d::Identity();

    static Eigen::Matrix4d get(const Eigen::Quaterniond&) {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = R_;
        return T;
    }

    // 设置旋转 (从YAML加载 R_gimbal2imubody)
    static void set_rotation(const Eigen::Matrix3d& R) { R_ = R; }
};

// Camera → Gimbal (相机外参，平移动态读取)
template<>
struct Transform<Frame::Camera, Frame::Gimbal> {
    inline static Eigen::Matrix3d R_ = Eigen::Matrix3d::Identity();

    static Eigen::Matrix4d get(const Eigen::Quaterniond&);  // 实现在cpp中

    static void set_rotation(const Eigen::Matrix3d& R) { R_ = R; }
};

// Barrel → Gimbal (枪口偏移，动态读取)
template<>
struct Transform<Frame::Barrel, Frame::Gimbal> {
    static Eigen::Matrix4d get(const Eigen::Quaterniond&);  // 实现在cpp中
};

// ============================================================================
// 4. 编译期路径推导
// ============================================================================

namespace detail {

// 向上遍历：From → 祖先 To
template<Frame From, Frame To>
struct ChainUp {
    static Eigen::Matrix4d get(const Eigen::Quaterniond& q) {
        if constexpr (From == To) {
            return Eigen::Matrix4d::Identity();
        } else {
            return ChainUp<Parent<From>::value, To>::get(q) * Transform<From>::get(q);
        }
    }
};

template<Frame F>
struct ChainUp<F, F> {
    static Eigen::Matrix4d get(const Eigen::Quaterniond&) {
        return Eigen::Matrix4d::Identity();
    }
};

// 计算到World的深度
template<Frame F, int Depth = 0>
struct DepthToWorld {
    static constexpr int value = (F == Frame::World)
        ? Depth
        : DepthToWorld<Parent<F>::value, Depth + 1>::value;
};

template<int Depth>
struct DepthToWorld<Frame::World, Depth> {
    static constexpr int value = Depth;
};

} // namespace detail

// ============================================================================
// 5. 变换矩阵接口
// ============================================================================

template<Frame From, Frame To>
Eigen::Matrix4d matrix(const Eigen::Quaterniond& q) {
    if constexpr (From == To) {
        return Eigen::Matrix4d::Identity();
    }

    constexpr int depth_from = detail::DepthToWorld<From>::value;
    constexpr int depth_to = detail::DepthToWorld<To>::value;

    if constexpr (depth_from >= depth_to) {
        if constexpr (To == Frame::World) {
            return detail::ChainUp<From, Frame::World>::get(q);
        } else if constexpr (Parent<From>::value == To) {
            return Transform<From>::get(q);
        } else {
            Eigen::Matrix4d T_from_to_world = detail::ChainUp<From, Frame::World>::get(q);
            Eigen::Matrix4d T_to_to_world = detail::ChainUp<To, Frame::World>::get(q);
            return T_to_to_world.inverse() * T_from_to_world;
        }
    } else {
        return matrix<To, From>(q).inverse();
    }
}

// ============================================================================
// 6. 坐标变换接口
// ============================================================================

// 变换点 (位置)
template<Frame From, Frame To>
Eigen::Vector3d point(const Eigen::Vector3d& p, const Eigen::Quaterniond& q) {
    if constexpr (From == To) {
        return p;
    } else {
        Eigen::Vector4d p_h(p.x(), p.y(), p.z(), 1.0);
        Eigen::Vector4d result = matrix<From, To>(q) * p_h;
        return result.head<3>();
    }
}

// 变换向量 (速度/方向，只应用旋转)
template<Frame From, Frame To>
Eigen::Vector3d vector(const Eigen::Vector3d& v, const Eigen::Quaterniond& q) {
    if constexpr (From == To) {
        return v;
    } else {
        Eigen::Matrix3d R = matrix<From, To>(q).template block<3, 3>(0, 0);
        return R * v;
    }
}

// ============================================================================
// 7. 初始化接口
// ============================================================================

/**
 * @brief 从YAML加载静态参数 (启动时调用一次)
 * - 相机内参、畸变系数
 * - R_gimbal2imubody (IMU安装偏差修正)
 * - R_camera2gimbal (相机安装角度)
 * @param yaml_file 文件名，默认 "camera.yaml"，自动加 CONFIG_DIR 前缀
 */
bool init(const std::string& yaml_file = "camera.yaml");

const cv::Mat& get_camera_matrix();
const cv::Mat& get_distort_coeffs();

// ============================================================================
// 8. 里程计位置追踪
// ============================================================================

/**
 * @brief 更新里程计位置
 * @param v_gimbal Gimbal坐标系下的速度 (vx前, vy左, vz上)
 * @param dt 时间间隔 (秒)
 * @param q_imu IMU四元数
 */
void update_odometry(const Eigen::Vector3d& v_gimbal, double dt, const Eigen::Quaterniond& q_imu);

/**
 * @brief 获取机器人在World中的位置 (里程计积分结果)
 */
const Eigen::Vector3d& get_robot_position();

/**
 * @brief 重置里程计位置为零
 */
void reset_odometry();

// ============================================================================
// 9. 便捷别名
// ============================================================================

inline Eigen::Vector3d cam_to_world(const Eigen::Vector3d& p, const Eigen::Quaterniond& q) {
    return point<Frame::Camera, Frame::World>(p, q);
}

/**
 * @brief 世界系目标点转换为“枪口原点 + 世界轴向”目标向量
 *
 * 输出向量用于弹道解算（对齐 rm.cv.fans 的 xyz_i_barrel 语义）：
 * - 原点在枪口
 * - 轴向仍是世界系 (x前 y左 z上)，不旋到枪口轴
 */
inline Eigen::Vector3d world_to_barrel_origin_world(
    const Eigen::Vector3d& p_world,
    const Eigen::Quaterniond& q
) {
    const Eigen::Vector3d barrel_origin_world = point<Frame::Barrel, Frame::World>(
        Eigen::Vector3d::Zero(), q
    );
    return p_world - barrel_origin_world;
}

/**
 * @brief 相机系目标点转换为“枪口原点 + 世界轴向”目标向量
 */
inline Eigen::Vector3d cam_to_barrel_origin_world(
    const Eigen::Vector3d& p_cam,
    const Eigen::Quaterniond& q
) {
    return world_to_barrel_origin_world(cam_to_world(p_cam, q), q);
}

inline Eigen::Vector3d world_to_barrel(const Eigen::Vector3d& p, const Eigen::Quaterniond& q) {
    // 完整坐标变换: 世界 -> 枪口坐标系 (含旋转+平移)
    // 若用于弹道解算，通常应优先使用 world_to_barrel_origin_world().
    return point<Frame::World, Frame::Barrel>(p, q);
}

inline Eigen::Vector3d cam_to_barrel(const Eigen::Vector3d& p, const Eigen::Quaterniond& q) {
    return point<Frame::Camera, Frame::Barrel>(p, q);
}

inline Eigen::Vector3d gimbal_to_world(const Eigen::Vector3d& p, const Eigen::Quaterniond& q) {
    return point<Frame::Gimbal, Frame::World>(p, q);
}

inline Eigen::Vector3d world_to_gimbal(const Eigen::Vector3d& p, const Eigen::Quaterniond& q) {
    return point<Frame::World, Frame::Gimbal>(p, q);
}

inline Eigen::Vector3d world_to_camera(const Eigen::Vector3d& p, const Eigen::Quaterniond& q) {
    return point<Frame::World, Frame::Camera>(p, q);
}

inline YpdCoord cam_to_world_ypd(const Eigen::Vector3d& p_cam, const Eigen::Quaterniond& q) {
    return aimer::math::xyz_to_ypd(cam_to_world(p_cam, q));
}

inline YpdCoord barrel_ypd(const Eigen::Vector3d& p_world, const Eigen::Quaterniond& q) {
    return aimer::math::xyz_to_ypd(world_to_barrel(p_world, q));
}

/**
 * @brief 将世界坐标点投影到图像像素坐标
 * @param p_world 世界坐标系中的点
 * @param q_imu IMU四元数
 * @param valid 输出参数，表示投影是否有效（点在相机前方）
 * @return 图像像素坐标
 */
inline cv::Point2f world_to_pixel(const Eigen::Vector3d& p_world, const Eigen::Quaterniond& q_imu, bool& valid) {
    // World → Camera
    Eigen::Vector3d p_cam = world_to_camera(p_world, q_imu);

    // 检查点是否在相机前方 (相机坐标系: z前)
    // Camera坐标系: x右, y下, z前
    if (p_cam.z() <= 0.1) {  // 至少10cm前方
        valid = false;
        return cv::Point2f(-1, -1);
    }

    // 投影到像素坐标
    const cv::Mat& K = get_camera_matrix();
    double fx = K.at<double>(0, 0);
    double fy = K.at<double>(1, 1);
    double cx = K.at<double>(0, 2);
    double cy = K.at<double>(1, 2);

    double u = fx * p_cam.x() / p_cam.z() + cx;
    double v = fy * p_cam.y() / p_cam.z() + cy;

    valid = true;
    return cv::Point2f(static_cast<float>(u), static_cast<float>(v));
}

} // namespace aimer::tf

#endif //RMCV_TRANSFORMER_HPP
