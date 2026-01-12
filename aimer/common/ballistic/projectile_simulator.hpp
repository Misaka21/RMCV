//
// 弹道模拟器 - 用于可视化枪管安装精度
//
// 功能:
//   1. 从枪管坐标系发射子弹
//   2. 模拟含空气阻力的弹道
//   3. 20Hz 发射频率
//   4. 用空心圆画出弹道轨迹
//   5. 标注发射时间戳
//

#ifndef RMCV_PROJECTILE_SIMULATOR_HPP
#define RMCV_PROJECTILE_SIMULATOR_HPP

#include <vector>
#include <deque>
#include <chrono>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "aimer/common/transformer/transformer.hpp"

namespace aimer::ballistic {

// ============================================================================
// 配置参数
// ============================================================================

struct BallisticConfig {
    double g = 9.8;                  // 重力加速度 (m/s²)
    double resistance_k = 0.01;       // 空气阻力系数
    double bullet_radius = 0.0085;    // 小弹丸半径 (m), 17mm
    double fire_rate_hz = 20.0;       // 发射频率 (Hz)
    double max_flight_time = 2.0;     // 最大飞行时间 (s)
    int max_bullets = 50;             // 最大同时显示子弹数
};

// ============================================================================
// 单个子弹状态
// ============================================================================

struct Bullet {
    double fire_time;                 // 发射时刻 (相对时间, s)
    double v0;                        // 初速度 (m/s)
    double aim_angle;                 // 发射仰角 (rad)
    Eigen::Vector3d direction;        // 发射方向 (Barrel 坐标系, 单位向量)
    Eigen::Quaterniond q_imu_fire;    // 发射时刻的 IMU 四元数

    // 获取子弹在 t 时刻的位置 (Barrel 坐标系)
    // 含空气阻力的弹道方程
    Eigen::Vector3d get_pos_barrel(double t, double g, double k) const {
        double dt = t - fire_time;
        if (dt < 0) return Eigen::Vector3d::Zero();

        double cos_a = std::cos(aim_angle);
        double sin_a = std::sin(aim_angle);

        // 水平距离 (沿 direction 在 XY 平面的投影)
        double w = dt * v0 * cos_a;

        // 含阻力的高度计算
        // h = (k*v0*sin(α) + g) * k*w / (k²*v0*cos(α)) + g*ln(1 - k*w/(v0*cos(α))) / k²
        double kv0cos = k * v0 * cos_a;
        double ratio = k * w / (v0 * cos_a);

        double h;
        if (std::abs(k) < 1e-6 || ratio >= 1.0) {
            // 无阻力或超出有效范围，使用简单抛物线
            h = w * sin_a / cos_a - 0.5 * g * dt * dt;
        } else {
            h = (k * v0 * sin_a + g) * k * w / (kv0cos * k)
              + g * std::log(1.0 - ratio) / (k * k);
        }

        // 水平方向单位向量 (在 XY 平面)
        Eigen::Vector2d dir_xy(direction.x(), direction.y());
        if (dir_xy.norm() < 1e-6) {
            dir_xy = Eigen::Vector2d(1, 0);  // 默认向前
        }
        dir_xy.normalize();

        return Eigen::Vector3d(w * dir_xy.x(), w * dir_xy.y(), h);
    }

    // 判断子弹是否已落地或超时
    bool is_expired(double current_time, double max_flight_time) const {
        return (current_time - fire_time) > max_flight_time;
    }
};

// ============================================================================
// 弹道模拟器
// ============================================================================

class ProjectileSimulator {
public:
    ProjectileSimulator() = default;

    void set_config(const BallisticConfig& cfg) { config_ = cfg; }

    // 更新弹速 (从串口获取)
    void set_bullet_speed(double v0) { bullet_speed_ = v0; }

    // 发射子弹 (从枪管坐标系原点发射)
    // aim_ypd: 瞄准点的 yaw/pitch (相对于枪管)
    void fire(double current_time, const Eigen::Quaterniond& q_imu) {
        // 限制发射频率
        double fire_interval = 1.0 / config_.fire_rate_hz;
        if (current_time - last_fire_time_ < fire_interval) {
            return;
        }
        last_fire_time_ = current_time;

        Bullet bullet;
        bullet.fire_time = current_time;
        bullet.v0 = bullet_speed_;
        bullet.aim_angle = 0;  // 平射
        bullet.direction = Eigen::Vector3d(1, 0, 0);  // 沿 X 轴 (前方)
        bullet.q_imu_fire = q_imu;

        bullets_.push_back(bullet);

        // 限制子弹数量
        while (bullets_.size() > static_cast<size_t>(config_.max_bullets)) {
            bullets_.pop_front();
        }
    }

    // 发射子弹 (指定方向和仰角)
    void fire(double current_time, const Eigen::Quaterniond& q_imu,
              const Eigen::Vector3d& direction, double aim_angle)
    {
        double fire_interval = 1.0 / config_.fire_rate_hz;
        if (current_time - last_fire_time_ < fire_interval) {
            return;
        }
        last_fire_time_ = current_time;

        Bullet bullet;
        bullet.fire_time = current_time;
        bullet.v0 = bullet_speed_;
        bullet.aim_angle = aim_angle;
        bullet.direction = direction.normalized();
        bullet.q_imu_fire = q_imu;

        bullets_.push_back(bullet);

        while (bullets_.size() > static_cast<size_t>(config_.max_bullets)) {
            bullets_.pop_front();
        }
    }

    // 清除过期子弹
    void cleanup(double current_time) {
        while (!bullets_.empty() &&
               bullets_.front().is_expired(current_time, config_.max_flight_time)) {
            bullets_.pop_front();
        }
    }

    // 绘制所有子弹到图像
    void draw(cv::Mat& image, double current_time, const Eigen::Quaterniond& q_imu_now) const {
        if (image.empty()) return;

        const cv::Mat& K = aimer::tf::get_camera_matrix();
        double fx = K.at<double>(0, 0);
        double fy = K.at<double>(1, 1);
        double cx = K.at<double>(0, 2);
        double cy = K.at<double>(1, 2);

        int text_y = 30;
        int bullet_idx = 0;

        for (const auto& bullet : bullets_) {
            double flight_time = current_time - bullet.fire_time;
            if (flight_time < 0) continue;

            // 获取子弹在 Barrel 坐标系的位置
            Eigen::Vector3d pos_barrel = bullet.get_pos_barrel(
                current_time, config_.g, config_.resistance_k);

            // Barrel → World: 使用发射时刻的姿态
            // 子弹离开枪管后，其世界位置由发射时刻的姿态决定
            // 飞行过程中只受重力和空气阻力影响，与云台后续运动无关
            Eigen::Vector3d pos_world = aimer::tf::gimbal_to_world(pos_barrel, bullet.q_imu_fire);

            // World → Camera: 使用当前时刻的姿态
            // 把子弹的世界位置投影到当前相机视角
            Eigen::Vector3d pos_cam = aimer::tf::world_to_camera(pos_world, q_imu_now);

            // 检查是否在相机前方
            if (pos_cam.z() <= 0.1) continue;

            // 投影到图像
            double u = fx * pos_cam.x() / pos_cam.z() + cx;
            double v = fy * pos_cam.y() / pos_cam.z() + cy;

            // 检查是否在图像内
            if (u < 0 || u >= image.cols || v < 0 || v >= image.rows) continue;

            // 计算子弹在图像上的半径
            double radius_world = config_.bullet_radius;
            double radius_pixel = fx * radius_world / pos_cam.z();
            radius_pixel = std::max(3.0, std::min(20.0, radius_pixel));

            // 根据飞行时间着色 (越老越暗)
            double alpha = 1.0 - flight_time / config_.max_flight_time;
            alpha = std::max(0.2, alpha);
            cv::Scalar color(0, 255 * alpha, 255 * alpha);  // 黄色渐变

            // 绘制空心圆
            cv::circle(image, cv::Point(static_cast<int>(u), static_cast<int>(v)),
                       static_cast<int>(radius_pixel), color, 2);

            // 绘制中心点
            cv::circle(image, cv::Point(static_cast<int>(u), static_cast<int>(v)),
                       2, color, -1);

            bullet_idx++;
        }

        // 在左侧绘制时间戳列表
        text_y = 30;
        cv::putText(image, fmt::format("Bullets: {}", bullets_.size()),
                    cv::Point(10, text_y), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(255, 255, 255), 1);
        text_y += 20;

        int count = 0;
        for (auto it = bullets_.rbegin(); it != bullets_.rend() && count < 10; ++it, ++count) {
            double flight_time = current_time - it->fire_time;
            std::string text = fmt::format("T-{:.2f}s: {:.1f}m/s",
                                           flight_time, it->v0);
            cv::putText(image, text, cv::Point(10, text_y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200, 200, 200), 1);
            text_y += 15;
        }
    }

    // 获取当前子弹数量
    size_t bullet_count() const { return bullets_.size(); }

    // 清空所有子弹
    void clear() { bullets_.clear(); }

private:
    BallisticConfig config_;
    double bullet_speed_ = 15.0;      // 默认弹速 (m/s)
    double last_fire_time_ = -1.0;
    std::deque<Bullet> bullets_;
};

} // namespace aimer::ballistic

#endif // RMCV_PROJECTILE_SIMULATOR_HPP
