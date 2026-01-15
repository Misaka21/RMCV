//
// IMU Gimbal2Imubody 交互式标定工具
// 3D 坐标轴可视化 + 实时 IMU 数据 + 键盘调整
//

#include <chrono>
#include <thread>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <fmt/format.h>
#include <fmt/color.h>

#include "hardware/hardware_node.hpp"
#include "hardware/serial/serial_thread.hpp"
#include "umt/umt.hpp"
#include "plugin/param/static_config.hpp"  // 用于读取 YAML
#include "aimer/common/transformer/transformer.hpp"
#include "aimer/common/robot_state.hpp"

using namespace std::chrono_literals;

// 从 camera.yaml 读取 R_gimbal2imubody
Eigen::Matrix3d load_gimbal2imubody_from_yaml(const std::string& yaml_file) {
    try {
        cv::FileStorage fs(yaml_file, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            fmt::print(fmt::fg(fmt::color::yellow),
                "[Warning] Cannot open {}, using identity matrix\n", yaml_file);
            return Eigen::Matrix3d::Identity();
        }

        cv::FileNode node = fs["R_gimbal2imubody"];
        if (node.empty() || !node.isSeq() || node.size() != 9) {
            fmt::print(fmt::fg(fmt::color::yellow),
                "[Warning] Invalid R_gimbal2imubody in {}, using identity matrix\n", yaml_file);
            fs.release();
            return Eigen::Matrix3d::Identity();
        }

        // 直接从 FileNode 序列读取
        std::vector<double> values;
        node >> values;
        fs.release();

        if (values.size() != 9) {
            fmt::print(fmt::fg(fmt::color::yellow),
                "[Warning] R_gimbal2imubody should have 9 values, got {}\n", values.size());
            return Eigen::Matrix3d::Identity();
        }

        // 转换到 Eigen::Matrix3d (行优先)
        Eigen::Matrix3d R;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                R(i, j) = values[i * 3 + j];
            }
        }

        fmt::print(fmt::fg(fmt::color::green), "[Loaded R_gimbal2imubody from {}]\n", yaml_file);
        fmt::print("  [ {:.6f}, {:.6f}, {:.6f},\n", R(0,0), R(0,1), R(0,2));
        fmt::print("    {:.6f}, {:.6f}, {:.6f},\n", R(1,0), R(1,1), R(1,2));
        fmt::print("    {:.6f}, {:.6f}, {:.6f} ]\n", R(2,0), R(2,1), R(2,2));

        return R;
    } catch (const std::exception& e) {
        fmt::print(fmt::fg(fmt::color::red),
            "[Error] Failed to load R_gimbal2imubody: {}\n", e.what());
        return Eigen::Matrix3d::Identity();
    }
}

// 状态跟踪结构
struct AxisRotationState {
    Eigen::Matrix3d base_matrix = Eigen::Matrix3d::Identity();  // 从 YAML 读取的初始矩阵
    int yaw_steps = 0;    // 绕 Z 轴旋转次数 (-n ~ +n, 每次 90°)
    int pitch_steps = 0;  // 绕 Y 轴旋转次数
    int roll_steps = 0;   // 绕 X 轴旋转次数

    // 设置初始矩阵
    void set_base_matrix(const Eigen::Matrix3d& R) {
        base_matrix = R;
    }

    // 获取增量旋转矩阵
    Eigen::Matrix3d get_delta_rotation() const {
        double yaw_angle = yaw_steps * M_PI / 2;
        double pitch_angle = pitch_steps * M_PI / 2;
        double roll_angle = roll_steps * M_PI / 2;

        Eigen::AngleAxisd yaw_rot(yaw_angle, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd pitch_rot(pitch_angle, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd roll_rot(roll_angle, Eigen::Vector3d::UnitX());

        return (yaw_rot * pitch_rot * roll_rot).toRotationMatrix();
    }

    // 获取最终旋转矩阵 = 增量 * 基础
    Eigen::Matrix3d get_rotation_matrix() const {
        return get_delta_rotation() * base_matrix;
    }

    void print_yaml() const {
        Eigen::Matrix3d R = get_rotation_matrix();
        fmt::print("\nR_gimbal2imubody: [ ");
        fmt::print("{:.6f}, {:.6f}, {:.6f}, ", R(0,0), R(0,1), R(0,2));
        fmt::print("{:.6f}, {:.6f}, {:.6f}, ", R(1,0), R(1,1), R(1,2));
        fmt::print("{:.6f}, {:.6f}, {:.6f} ]\n", R(2,0), R(2,1), R(2,2));
    }
};

// 3D 点投影到 2D（等距投影 + 旋转视角）
cv::Point2f project_3d_to_2d(const Eigen::Vector3d& p3d, int cx, int cy, double scale) {
    // 从斜后上方看（更立体的视角）
    // 视角旋转：绕 Y 轴旋转 -30°，绕 X 轴旋转 -20°

    double angle_y = -30.0 * M_PI / 180.0;  // 绕 Y 轴（水平旋转）
    double angle_x = 0 * M_PI / 180.0;  // 绕 X 轴（俯视）

    // 应用视角旋转矩阵
    Eigen::Matrix3d R_view;
    R_view = Eigen::AngleAxisd(angle_y, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(angle_x, Eigen::Vector3d::UnitX());

    Eigen::Vector3d p_view = R_view * p3d;

    // 等距投影（深度影响大小）
    double depth_scale = 1.0 / (1.0 + p_view.x() * 0.15);  // X 越大（越远）越小

    // 修正：Y轴(左)应该映射到屏幕左侧，所以用负号
    float x_2d = cx - p_view.y() * scale * depth_scale;      // Y → 屏幕 -X (左边)
    float y_2d = cy - p_view.z() * scale * depth_scale;      // Z → 屏幕 -Y (上边)

    return cv::Point2f(x_2d, y_2d);
}

// 绘制 3D 坐标轴
void draw_gimbal_axes_3d(cv::Mat& canvas,
                         const AxisRotationState& state,
                         const serial::SerialReceiveData& imu_data,
                         const Eigen::Quaterniond& q_imu) {
    int cx = canvas.cols / 2;
    int cy = canvas.rows / 2 + 50;  // 稍微下移，留出上方空间
    double scale = 180.0;

    // 使用 transformer API 进行坐标变换
    // transformer 内部已经应用了 state 中的 R_gimbal2imubody

    // 绘制 Gimbal 坐标系网格框架
    cv::Scalar grid_color(60, 60, 60);
    cv::Scalar grid_highlight(100, 100, 100);

    // 定义网格范围和间隔
    double grid_size = 0.2;  // 网格间隔 20cm
    double range = 1.0;      // 网格范围 ±1m

    // 绘制 XY 平面网格 (Z=0, 水平面)
    for (double x = -range; x <= range; x += grid_size) {
        for (double y = -range; y <= range; y += grid_size) {
            Eigen::Vector3d p1(x, y, 0);
            Eigen::Vector3d p2(x + grid_size, y, 0);
            Eigen::Vector3d p3(x, y + grid_size, 0);

            // 应用云台旋转: Gimbal → World
            Eigen::Vector3d p1_world = aimer::tf::gimbal_to_world(p1, q_imu);
            Eigen::Vector3d p2_world = aimer::tf::gimbal_to_world(p2, q_imu);
            Eigen::Vector3d p3_world = aimer::tf::gimbal_to_world(p3, q_imu);

            cv::Point2f pt1 = project_3d_to_2d(p1_world, cx, cy, scale);
            cv::Point2f pt2 = project_3d_to_2d(p2_world, cx, cy, scale);
            cv::Point2f pt3 = project_3d_to_2d(p3_world, cx, cy, scale);

            // 主轴线加粗
            cv::Scalar color = grid_color;
            int thickness = 1;
            if (std::abs(x) < 1e-3 || std::abs(y) < 1e-3) {
                color = grid_highlight;
                thickness = 2;
            }

            cv::line(canvas, pt1, pt2, color, thickness, cv::LINE_AA);
            cv::line(canvas, pt1, pt3, color, thickness, cv::LINE_AA);
        }
    }

    // 绘制 XZ 平面网格 (Y=0, 正面)
    for (double x = -range; x <= range; x += grid_size) {
        for (double z = -range; z <= range; z += grid_size) {
            Eigen::Vector3d p1(x, 0, z);
            Eigen::Vector3d p2(x + grid_size, 0, z);
            Eigen::Vector3d p3(x, 0, z + grid_size);

            Eigen::Vector3d p1_world = aimer::tf::gimbal_to_world(p1, q_imu);
            Eigen::Vector3d p2_world = aimer::tf::gimbal_to_world(p2, q_imu);
            Eigen::Vector3d p3_world = aimer::tf::gimbal_to_world(p3, q_imu);

            cv::Point2f pt1 = project_3d_to_2d(p1_world, cx, cy, scale);
            cv::Point2f pt2 = project_3d_to_2d(p2_world, cx, cy, scale);
            cv::Point2f pt3 = project_3d_to_2d(p3_world, cx, cy, scale);

            cv::Scalar color = grid_color;
            int thickness = 1;
            if (std::abs(x) < 1e-3 || std::abs(z) < 1e-3) {
                color = grid_highlight;
                thickness = 2;
            }

            cv::line(canvas, pt1, pt2, color, thickness, cv::LINE_AA);
            cv::line(canvas, pt1, pt3, color, thickness, cv::LINE_AA);
        }
    }

    // 绘制 YZ 平面网格 (X=0, 侧面)
    for (double y = -range; y <= range; y += grid_size) {
        for (double z = -range; z <= range; z += grid_size) {
            Eigen::Vector3d p1(0, y, z);
            Eigen::Vector3d p2(0, y + grid_size, z);
            Eigen::Vector3d p3(0, y, z + grid_size);

            Eigen::Vector3d p1_world = aimer::tf::gimbal_to_world(p1, q_imu);
            Eigen::Vector3d p2_world = aimer::tf::gimbal_to_world(p2, q_imu);
            Eigen::Vector3d p3_world = aimer::tf::gimbal_to_world(p3, q_imu);

            cv::Point2f pt1 = project_3d_to_2d(p1_world, cx, cy, scale);
            cv::Point2f pt2 = project_3d_to_2d(p2_world, cx, cy, scale);
            cv::Point2f pt3 = project_3d_to_2d(p3_world, cx, cy, scale);

            cv::Scalar color = grid_color;
            int thickness = 1;
            if (std::abs(y) < 1e-3 || std::abs(z) < 1e-3) {
                color = grid_highlight;
                thickness = 2;
            }

            cv::line(canvas, pt1, pt2, color, thickness, cv::LINE_AA);
            cv::line(canvas, pt1, pt3, color, thickness, cv::LINE_AA);
        }
    }

    // Gimbal 坐标系的三个基向量（加长以便看清）
    Eigen::Vector3d x_axis(1.2, 0, 0);  // X - 前
    Eigen::Vector3d y_axis(0, 1.2, 0);  // Y - 左
    Eigen::Vector3d z_axis(0, 0, 1.2);  // Z - 上

    // 应用云台旋转: Gimbal → World
    Eigen::Vector3d x_world = aimer::tf::gimbal_to_world(x_axis, q_imu);
    Eigen::Vector3d y_world = aimer::tf::gimbal_to_world(y_axis, q_imu);
    Eigen::Vector3d z_world = aimer::tf::gimbal_to_world(z_axis, q_imu);

    // 投影到 2D
    Eigen::Vector3d origin(0, 0, 0);
    cv::Point2f origin_2d = project_3d_to_2d(origin, cx, cy, scale);

    cv::Point2f x_end = project_3d_to_2d(x_world, cx, cy, scale);
    cv::Point2f y_end = project_3d_to_2d(y_world, cx, cy, scale);
    cv::Point2f z_end = project_3d_to_2d(z_world, cx, cy, scale);

    // 8. 绘制坐标轴阴影（增强立体感）
    cv::Scalar shadow_color(20, 20, 20);
    cv::Point2f shadow_offset(3, 3);

    cv::arrowedLine(canvas, origin_2d + shadow_offset, x_end + shadow_offset,
                   shadow_color, 5, cv::LINE_AA, 0, 0.2);
    cv::arrowedLine(canvas, origin_2d + shadow_offset, y_end + shadow_offset,
                   shadow_color, 5, cv::LINE_AA, 0, 0.2);
    cv::arrowedLine(canvas, origin_2d + shadow_offset, z_end + shadow_offset,
                   shadow_color, 5, cv::LINE_AA, 0, 0.2);

    // 9. 绘制坐标轴（粗箭头 + 渐变效果）
    // X 轴 - 红色（前）- 最粗最亮
    cv::arrowedLine(canvas, origin_2d, x_end,
                   cv::Scalar(0, 0, 255), 5, cv::LINE_AA, 0, 0.2);
    cv::circle(canvas, x_end, 8, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
    cv::putText(canvas, "X-FRONT", x_end + cv::Point2f(15, 5),
               cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 100, 255), 2, cv::LINE_AA);

    // Y 轴 - 蓝色（左）
    cv::arrowedLine(canvas, origin_2d, y_end,
                   cv::Scalar(255, 0, 0), 5, cv::LINE_AA, 0, 0.2);
    cv::circle(canvas, y_end, 8, cv::Scalar(255, 0, 0), -1, cv::LINE_AA);
    cv::putText(canvas, "Y-LEFT", y_end + cv::Point2f(15, 5),
               cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 100, 0), 2, cv::LINE_AA);

    // Z 轴 - 黄色（上）
    cv::arrowedLine(canvas, origin_2d, z_end,
                   cv::Scalar(0, 255, 255), 5, cv::LINE_AA, 0, 0.2);
    cv::circle(canvas, z_end, 8, cv::Scalar(0, 255, 255), -1, cv::LINE_AA);
    cv::putText(canvas, "Z-UP", z_end + cv::Point2f(15, 5),
               cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(100, 255, 255), 2, cv::LINE_AA);

    // 10. 绘制原点标记
    cv::circle(canvas, origin_2d, 10, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
    cv::circle(canvas, origin_2d, 3, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);

    // 11. 添加视角说明
    cv::putText(canvas, "Gimbal Frame (X-front, Y-left, Z-up)",
               cv::Point(20, canvas.rows - 40),
               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(100, 100, 100), 1, cv::LINE_AA);
}

// 绘制信息文本
void draw_info_text(cv::Mat& canvas,
                   const AxisRotationState& state,
                   const serial::SerialReceiveData& imu_data) {
    int y = 30;
    int line_height = 30;

    // 标题
    cv::putText(canvas, "IMU Gimbal2Imubody Calibration Tool",
               cv::Point(20, y), cv::FONT_HERSHEY_SIMPLEX,
               0.8, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
    y += line_height + 10;

    // 分隔线
    cv::line(canvas, cv::Point(20, y), cv::Point(canvas.cols - 20, y),
            cv::Scalar(100, 100, 100), 1);
    y += 20;

    // IMU 原始数据
    std::string imu_info = fmt::format("IMU Raw: yaw={:.1f}°  pitch={:.1f}°  roll={:.1f}°",
                                       imu_data.yaw, imu_data.pitch, imu_data.roll);
    cv::putText(canvas, imu_info, cv::Point(20, y),
               cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(150, 150, 150), 1, cv::LINE_AA);
    y += line_height;

    // 当前旋转状态
    std::string steps_info = fmt::format("Rotation: Yaw={:+d}x90°  Pitch={:+d}x90°  Roll={:+d}x90°",
                                        state.yaw_steps, state.pitch_steps, state.roll_steps);
    cv::putText(canvas, steps_info, cv::Point(20, y),
               cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    y += line_height;

    // R_gimbal2imubody 矩阵
    Eigen::Matrix3d R = state.get_rotation_matrix();

    cv::putText(canvas, "R_gimbal2imubody:", cv::Point(20, y),
               cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 1, cv::LINE_AA);
    y += line_height;

    for (int i = 0; i < 3; ++i) {
        std::string row = fmt::format("  [ {: .4f}, {: .4f}, {: .4f} ]",
                                     R(i, 0), R(i, 1), R(i, 2));
        cv::putText(canvas, row, cv::Point(20, y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
        y += 25;
    }

    y += 10;

    // 操作提示
    cv::putText(canvas, "Controls: q/w(Yaw)  a/s(Pitch)  z/x(Roll)  r(Reset)  ESC(Exit)",
               cv::Point(20, canvas.rows - 20),
               cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(100, 200, 255), 1, cv::LINE_AA);
}

// 处理键盘输入
void handle_keyboard(int key, AxisRotationState& state) {
    bool changed = false;

    switch (key) {
        case 'q': case 'Q':
            state.yaw_steps++;
            changed = true;
            fmt::print(fmt::fg(fmt::color::cyan), "Yaw +90° (steps={})\n", state.yaw_steps);
            break;
        case 'w': case 'W':
            state.yaw_steps--;
            changed = true;
            fmt::print(fmt::fg(fmt::color::cyan), "Yaw -90° (steps={})\n", state.yaw_steps);
            break;
        case 'a': case 'A':
            state.pitch_steps++;
            changed = true;
            fmt::print(fmt::fg(fmt::color::green), "Pitch +90° (steps={})\n", state.pitch_steps);
            break;
        case 's': case 'S':
            state.pitch_steps--;
            changed = true;
            fmt::print(fmt::fg(fmt::color::green), "Pitch -90° (steps={})\n", state.pitch_steps);
            break;
        case 'z': case 'Z':
            state.roll_steps++;
            changed = true;
            fmt::print(fmt::fg(fmt::color::yellow), "Roll +90° (steps={})\n", state.roll_steps);
            break;
        case 'x': case 'X':
            state.roll_steps--;
            changed = true;
            fmt::print(fmt::fg(fmt::color::yellow), "Roll -90° (steps={})\n", state.roll_steps);
            break;
        case 'r': case 'R':
            state.yaw_steps = 0;
            state.pitch_steps = 0;
            state.roll_steps = 0;
            changed = true;
            fmt::print(fmt::fg(fmt::color::red), "Reset to identity\n");
            break;
    }

    if (changed) {
        state.print_yaml();
    }
}

int main() {
    fmt::print(fmt::fg(fmt::color::gold),
        "====================================================================\n"
        "           IMU Gimbal2Imubody Interactive Calibration Tool         \n"
        "====================================================================\n"
        "View: Looking from BACK to FRONT\n"
        "  X-axis (Red)    : Forward (into screen)\n"
        "  Y-axis (Blue)   : Left to Right\n"
        "  Z-axis (Yellow) : Down to Up\n"
        "\n"
        "Usage:\n"
        "  1. Move your gimbal (look up/down, turn left/right, tilt)\n"
        "  2. Observe if the 3D axes match real gimbal movement\n"
        "  3. If not, press q/w/a/s/z/x to adjust axis mapping\n"
        "  4. When axes behavior is correct, copy R_gimbal2imubody to config/camera.yaml\n"
        "\n"
        "Controls:\n"
        "  q/w - Yaw ±90°\n"
        "  a/s - Pitch ±90°\n"
        "  z/x - Roll ±90°\n"
        "  r   - Reset\n"
        "  ESC - Exit\n"
        "====================================================================\n\n"
    );

    // 初始化坐标变换系统
    if (!aimer::tf::init("camera.yaml")) {
        fmt::print(fmt::fg(fmt::color::red), "[错误] 无法加载 camera.yaml\n");
        return 1;
    }
    fmt::print(fmt::fg(fmt::color::green), "[坐标变换系统已初始化]\n");

    // 启动硬件节点
    std::thread hw_thread([]() {
        hardware::start_hardware_node();
    });
    hw_thread.detach();

    // 等待硬件就绪
    auto hardware_ready = umt::BasicObjManager<bool>::find_or_create("hardware_running", false);
    while (!hardware_ready->get()) {
        fmt::print(fmt::fg(fmt::color::yellow), "\r[Waiting for hardware...]");
        std::fflush(stdout);
        std::this_thread::sleep_for(100ms);
    }
    fmt::print(fmt::fg(fmt::color::green), "\n[Hardware ready]\n");

    // 订阅同步帧
    umt::Subscriber<hardware::SyncFrame> sub("sync_frame");

    // 初始化状态并加载 YAML
    AxisRotationState state;

    // 读取 camera.yaml 中的初始 R_gimbal2imubody
    std::string yaml_path = std::string(CONFIG_DIR) + "/camera.yaml";
    Eigen::Matrix3d initial_R = load_gimbal2imubody_from_yaml(yaml_path);
    state.set_base_matrix(initial_R);

    fmt::print(fmt::fg(fmt::color::cyan),
        "\n[Initial state: steps=(0,0,0), you can press q/w/a/s/z/x to adjust]\n\n");

    cv::namedWindow("IMU Calibration", cv::WINDOW_NORMAL);
    cv::resizeWindow("IMU Calibration", 1200, 800);

    // 主循环
    while (true) {
        try {
            auto frame = sub.pop_for(100);

            if (!frame.serial_valid) {
                continue;
            }

            // 更新 transformer 中的 R_gimbal2imubody 为当前调整后的矩阵
            aimer::tf::Transform<aimer::tf::Frame::Gimbal, aimer::tf::Frame::Imu>::set_rotation(
                state.get_rotation_matrix()
            );

            // 创建 RobotState 并设置 q_imu
            aimer::RobotState robot_state;
            robot_state.set_euler(frame.serial_data.yaw,
                                 frame.serial_data.pitch,
                                 frame.serial_data.roll);

            // 创建黑色画布
            cv::Mat canvas(800, 1200, CV_8UC3, cv::Scalar(0, 0, 0));

            // 绘制 3D 坐标轴 (传入 q_imu)
            draw_gimbal_axes_3d(canvas, state, frame.serial_data, robot_state.q_imu);

            // 绘制信息文本
            draw_info_text(canvas, state, frame.serial_data);

            // 显示
            cv::imshow("IMU Calibration", canvas);

            // 键盘处理
            int key = cv::waitKey(1) & 0xFF;

            if (key == 27) {  // ESC
                fmt::print(fmt::fg(fmt::color::green), "\n[Exiting...]\n");
                state.print_yaml();
                break;
            }

            handle_keyboard(key, state);

        } catch (const umt::MessageError_Timeout&) {
            // 超时，继续等待
        }
    }

    cv::destroyAllWindows();
    return 0;
}
