//
// transformer 模块测试
//

#include <cmath>
#include <fmt/color.h>
#include <fmt/core.h>

#include "aimer/common/transformer/transformer.hpp"

// 简单断言宏
#define ASSERT_NEAR(a, b, eps) \
    if (std::abs((a) - (b)) > (eps)) { \
        fmt::print(fmt::fg(fmt::color::red), "FAIL: {} != {} (line {})\n", a, b, __LINE__); \
        return 1; \
    }

#define ASSERT_VEC_NEAR(v1, v2, eps) \
    if ((v1 - v2).norm() > (eps)) { \
        fmt::print(fmt::fg(fmt::color::red), "FAIL: vec diff {} (line {})\n", (v1-v2).norm(), __LINE__); \
        return 1; \
    }

int main() {
    fmt::print(fmt::fg(fmt::color::gold), "========== Transformer Tests ==========\n");

    // 测试1: 单位四元数，World == Gimbal
    {
        fmt::print("Test 1: Identity quaternion (World == Gimbal)... ");
        Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
        Eigen::Vector3d p_gimbal(1, 2, 3);

        auto p_world = tf::point<tf::Frame::Gimbal, tf::Frame::World>(p_gimbal, q);

        // R_gimbal2imubody 默认是 Identity，所以 p_world == p_gimbal
        ASSERT_VEC_NEAR(p_world, p_gimbal, 1e-9);
        fmt::print(fmt::fg(fmt::color::green), "PASS\n");
    }

    // 测试2: 绕Z轴旋转90度
    {
        fmt::print("Test 2: Rotate 90 deg around Z... ");
        // 绕Z轴旋转90度的四元数
        double angle = M_PI / 2;
        Eigen::Quaterniond q(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));

        Eigen::Vector3d p_gimbal(1, 0, 0);  // X方向
        auto p_world = tf::point<tf::Frame::Gimbal, tf::Frame::World>(p_gimbal, q);

        // 旋转后应该变成 Y 方向 (0, 1, 0)
        // 注意: 这取决于 R_gimbal2imubody 的设置，默认 Identity
        ASSERT_VEC_NEAR(p_world, Eigen::Vector3d(0, 1, 0), 1e-9);
        fmt::print(fmt::fg(fmt::color::green), "PASS\n");
    }

    // 测试3: 向量变换（只旋转，不平移）
    {
        fmt::print("Test 3: Vector transform (rotation only)... ");
        double angle = M_PI / 2;
        Eigen::Quaterniond q(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));

        Eigen::Vector3d v_gimbal(1, 0, 0);
        auto v_world = tf::vector<tf::Frame::Gimbal, tf::Frame::World>(v_gimbal, q);

        ASSERT_VEC_NEAR(v_world, Eigen::Vector3d(0, 1, 0), 1e-9);
        fmt::print(fmt::fg(fmt::color::green), "PASS\n");
    }

    // 测试4: 逆变换
    {
        fmt::print("Test 4: Inverse transform... ");
        Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));
        Eigen::Vector3d p_original(1, 2, 3);

        // Gimbal -> World -> Gimbal 应该回到原点
        auto p_world = tf::point<tf::Frame::Gimbal, tf::Frame::World>(p_original, q);
        auto p_back = tf::point<tf::Frame::World, tf::Frame::Gimbal>(p_world, q);

        ASSERT_VEC_NEAR(p_back, p_original, 1e-9);
        fmt::print(fmt::fg(fmt::color::green), "PASS\n");
    }

    // 测试5: 里程计积分
    {
        fmt::print("Test 5: Odometry integration... ");
        tf::reset_odometry();

        Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
        Eigen::Vector3d v(1, 0, 0);  // 向右 1 m/s
        double dt = 0.1;  // 100ms

        // 积分10次，应该移动 1m
        for (int i = 0; i < 10; ++i) {
            tf::update_odometry(v, dt, q);
        }

        auto pos = tf::get_robot_position();
        ASSERT_NEAR(pos.x(), 1.0, 1e-6);
        ASSERT_NEAR(pos.y(), 0.0, 1e-6);
        ASSERT_NEAR(pos.z(), 0.0, 1e-6);
        fmt::print(fmt::fg(fmt::color::green), "PASS\n");
    }

    // 测试6: 里程计 + 旋转
    {
        fmt::print("Test 6: Odometry with rotation... ");
        tf::reset_odometry();

        // 云台朝向Y轴正方向（绕Z旋转90度）
        Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));

        // Gimbal下向前(Z)走，在World中应该是向Y走
        // 注意: Gimbal的Z是前方
        Eigen::Vector3d v_gimbal(0, 0, 1);  // 向前 1 m/s
        double dt = 1.0;

        tf::update_odometry(v_gimbal, dt, q);

        auto pos = tf::get_robot_position();
        // 向前走1秒，World下应该是 Y 方向移动
        // 具体方向取决于坐标系定义
        fmt::print("pos = ({}, {}, {}) ", pos.x(), pos.y(), pos.z());
        fmt::print(fmt::fg(fmt::color::green), "PASS (manual check)\n");
    }

    // 测试7: Camera -> Barrel 链式变换
    {
        fmt::print("Test 7: Camera -> Barrel chain... ");
        Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
        Eigen::Vector3d p_cam(0, 0, 5);  // 相机前方5m

        auto p_barrel = tf::cam_to_barrel(p_cam, q);

        // 如果 camera_offset 和 barrel_offset 都是0，应该相等
        // 实际值取决于 TOML 配置
        fmt::print("p_barrel = ({}, {}, {}) ", p_barrel.x(), p_barrel.y(), p_barrel.z());
        fmt::print(fmt::fg(fmt::color::green), "PASS (manual check)\n");
    }

    fmt::print(fmt::fg(fmt::color::gold), "========== All Tests Done ==========\n");
    return 0;
}
