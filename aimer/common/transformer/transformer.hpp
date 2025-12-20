//
// Created by nuc11 on 2025/10/5.
// TF树坐标变换系统 - 编译期自动推导变换链
//

#ifndef RMCV_TRANSFORMER_HPP
#define RMCV_TRANSFORMER_HPP

#include <Eigen/Dense>
#include <type_traits>

#include "aimer/common/math/math.hpp"

namespace tf {

// 使用 math 命名空间的类型
using math::YpdCoord;

// ============================================================================
// 1. 坐标系定义
// ============================================================================

enum class Frame {
    World,   // 大地坐标系 (预测在此进行)
    Imu,     // 云台坐标系 (IMU安装位置)
    Camera,  // 相机坐标系 (PnP结果)
    Barrel   // 枪口坐标系 (弹道起点)
};

// ============================================================================
// 2. TF树结构定义 (编译期)
//
//   World (大地坐标系)
//     │
//     └── Imu (云台坐标系，动态旋转)
//          │
//          ├── Camera (静态标定)
//          │
//          └── Barrel (静态偏移)
//
// ============================================================================

// 父节点关系
template<Frame F> struct Parent;
template<> struct Parent<Frame::Imu>    { static constexpr Frame value = Frame::World; };
template<> struct Parent<Frame::Camera> { static constexpr Frame value = Frame::Imu; };
template<> struct Parent<Frame::Barrel> { static constexpr Frame value = Frame::Imu; };

// World没有父节点
template<> struct Parent<Frame::World> { static constexpr Frame value = Frame::World; };

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
        T.block<3,3>(0,0) = q_imu.toRotationMatrix();
        return T;
    }
};

// Camera → Imu (静态变换，相机外参标定)
template<>
struct Transform<Frame::Camera, Frame::Imu> {
    inline static Eigen::Matrix4d T_ = Eigen::Matrix4d::Identity();

    static Eigen::Matrix4d get(const Eigen::Quaterniond&) { return T_; }

    // 初始化接口
    static void init(const Eigen::Matrix3d& R, const Eigen::Vector3d& t) {
        T_.block<3,3>(0,0) = R;
        T_.block<3,1>(0,3) = t;
    }
    static void init(const Eigen::Matrix4d& T) { T_ = T; }
};

// Barrel → Imu (静态变换，枪口偏移)
template<>
struct Transform<Frame::Barrel, Frame::Imu> {
    inline static Eigen::Matrix4d T_ = Eigen::Matrix4d::Identity();

    static Eigen::Matrix4d get(const Eigen::Quaterniond&) { return T_; }

    // 初始化接口 (通常只有平移)
    static void init(const Eigen::Vector3d& offset) {
        T_ = Eigen::Matrix4d::Identity();
        T_.block<3,1>(0,3) = offset;
    }
    static void init(const Eigen::Matrix3d& R, const Eigen::Vector3d& t) {
        T_.block<3,3>(0,0) = R;
        T_.block<3,1>(0,3) = t;
    }
};

// ============================================================================
// 4. 编译期路径推导
// ============================================================================

namespace detail {

// 判断 A 是否是 B 的祖先
template<Frame A, Frame B, typename Enable = void>
struct IsAncestor : std::false_type {};

template<Frame A, Frame B>
struct IsAncestor<A, B, std::enable_if_t<!std::is_same_v<Frame, Frame> && (A != B) && (B != Frame::World)>> {
    static constexpr bool value =
        (A == Parent<B>::value) || IsAncestor<A, Parent<B>::value>::value;
};

template<Frame A>
struct IsAncestor<A, Frame::World> : std::false_type {};

template<Frame A>
struct IsAncestor<A, A> : std::false_type {};

// 向上遍历：From → 祖先 To
template<Frame From, Frame To>
struct ChainUp {
    static Eigen::Matrix4d get(const Eigen::Quaterniond& q) {
        if constexpr (From == To) {
            return Eigen::Matrix4d::Identity();
        } else {
            // T_to_from = T_to_parent * T_parent_from
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

// 找公共祖先的深度
template<Frame F, int Depth = 0>
struct DepthToWorld {
    static constexpr int value = (F == Frame::World) ? Depth : DepthToWorld<Parent<F>::value, Depth + 1>::value;
};

template<int Depth>
struct DepthToWorld<Frame::World, Depth> {
    static constexpr int value = Depth;
};

} // namespace detail

// ============================================================================
// 5. 变换矩阵接口
// ============================================================================

/**
 * @brief 获取从 From 到 To 的变换矩阵
 * @tparam From 源坐标系
 * @tparam To 目标坐标系
 * @param q 云台IMU四元数
 * @return 4x4齐次变换矩阵
 */
template<Frame From, Frame To>
Eigen::Matrix4d matrix(const Eigen::Quaterniond& q) {
    if constexpr (From == To) {
        return Eigen::Matrix4d::Identity();
    }

    // 计算两个坐标系到World的深度
    constexpr int depth_from = detail::DepthToWorld<From>::value;
    constexpr int depth_to = detail::DepthToWorld<To>::value;

    if constexpr (depth_from >= depth_to) {
        if constexpr (From == Frame::World) {
            return detail::ChainUp<To, Frame::World>::get(q).inverse();
        } else if constexpr (To == Frame::World) {
            return detail::ChainUp<From, Frame::World>::get(q);
        } else if constexpr (Parent<From>::value == To) {
            return Transform<From>::get(q);
        } else {
            // 通过World中转
            Eigen::Matrix4d T_from_to_world = detail::ChainUp<From, Frame::World>::get(q);
            Eigen::Matrix4d T_to_to_world = detail::ChainUp<To, Frame::World>::get(q);
            return T_to_to_world.inverse() * T_from_to_world;
        }
    } else {
        return matrix<To, From>(q).inverse();
    }
}

// ============================================================================
// 6. 直接输出坐标接口
// ============================================================================

/**
 * @brief 变换点 (位置，应用旋转+平移)
 */
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

/**
 * @brief 变换向量 (速度/方向，只应用旋转)
 */
template<Frame From, Frame To>
Eigen::Vector3d vector(const Eigen::Vector3d& v, const Eigen::Quaterniond& q) {
    if constexpr (From == To) {
        return v;
    } else {
        Eigen::Matrix3d R = matrix<From, To>(q).block<3,3>(0,0);
        return R * v;
    }
}

// ============================================================================
// 7. 初始化接口
// ============================================================================

/**
 * @brief 初始化静态变换参数
 */
inline void init(const Eigen::Matrix3d& R_cam_imu,
                 const Eigen::Vector3d& t_cam_imu,
                 const Eigen::Vector3d& t_barrel_imu) {
    Transform<Frame::Camera, Frame::Imu>::init(R_cam_imu, t_cam_imu);
    Transform<Frame::Barrel, Frame::Imu>::init(t_barrel_imu);
}

// ============================================================================
// 8. 便捷别名
// ============================================================================

inline Eigen::Vector3d cam_to_world(const Eigen::Vector3d& p, const Eigen::Quaterniond& q) {
    return point<Frame::Camera, Frame::World>(p, q);
}

inline Eigen::Vector3d world_to_barrel(const Eigen::Vector3d& p, const Eigen::Quaterniond& q) {
    return point<Frame::World, Frame::Barrel>(p, q);
}

inline Eigen::Vector3d cam_to_barrel(const Eigen::Vector3d& p, const Eigen::Quaterniond& q) {
    return point<Frame::Camera, Frame::Barrel>(p, q);
}

inline Eigen::Vector3d imu_vel_to_world(const Eigen::Vector3d& v, const Eigen::Quaterniond& q) {
    return vector<Frame::Imu, Frame::World>(v, q);
}

// xyz→ypd 组合 (使用 math::xyz_to_ypd)
inline YpdCoord cam_to_world_ypd(const Eigen::Vector3d& p_cam, const Eigen::Quaterniond& q) {
    return math::xyz_to_ypd(cam_to_world(p_cam, q));
}

inline YpdCoord barrel_ypd(const Eigen::Vector3d& p_world, const Eigen::Quaterniond& q) {
    return math::xyz_to_ypd(world_to_barrel(p_world, q));
}

} // namespace tf

#endif //RMCV_TRANSFORMER_HPP
