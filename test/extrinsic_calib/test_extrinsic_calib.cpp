//
// 相机外参自动标定测试程序
//
// 使用方法:
//   1. 将棋盘格标定板固定在某个位置 (不要动!)
//   2. 启动程序，云台对准标定板
//   3. 按 SPACE 拍照，同时转动云台到不同角度
//   4. 采集 20-50 组数据后按 'c' 开始标定
//   5. 标定完成后将结果写入配置文件
//
// 快捷键:
//   SPACE - 拍照采样
//   'c'   - 开始标定
//   'g'   - 只用网格搜索 (调试用)
//   'r'   - 重置所有采样
//   'd'   - 删除最后一个采样
//   's'   - 保存采样到文件
//   'l'   - 从文件加载采样
//   'q'   - 退出
//

#include <atomic>
#include <chrono>
#include <deque>
#include <fstream>
#include <mutex>
#include <thread>
#include <vector>

#include <fmt/format.h>
#include <fmt/color.h>
#include <opencv2/opencv.hpp>

#include "hardware/hik_cam/hik_camera.hpp"
#include "hardware/serial/serial_thread.hpp"
#include "aimer/common/transformer/transformer.hpp"
#include "plugin/param/static_config.hpp"
#include "plugin/param/runtime_parameter.hpp"
#include "plugin/debug/logger.hpp"
#include "umt/umt.hpp"

#include "extrinsic_calib.hpp"

// 尝试包含Ceres版本 (可选)
#if __has_include(<ceres/ceres.h>)
    #include "extrinsic_calib_ceres.hpp"
    #define HAS_CERES 1
#else
    #define HAS_CERES 0
#endif

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

// ============================================================================
// 配置参数
// ============================================================================

// 棋盘格配置 (内角点数)
constexpr int BOARD_WIDTH = 11;   // 横向内角点数
constexpr int BOARD_HEIGHT = 8;   // 纵向内角点数
constexpr float SQUARE_SIZE = 0.020f;  // 每格边长 (m) = 20mm

constexpr size_t MIN_SAMPLES = 10;     // 最少采样数
constexpr size_t MAX_IMU_BUFFER = 500; // IMU缓冲区大小

// ============================================================================
// 全局状态
// ============================================================================

std::atomic<bool> g_running{true};

// IMU数据 (最新一帧)
std::mutex g_imu_mutex;
Eigen::Quaterniond g_q_imu = Eigen::Quaterniond::Identity();
bool g_imu_valid = false;

// 采样数据
std::mutex g_samples_mutex;
std::vector<extrinsic_calib::CalibSample> g_samples;

// ============================================================================
// IMU接收线程
// ============================================================================

void imu_receiver_thread() {
    try {
        serial::start_serial_communication();  // 从配置文件读取
        std::this_thread::sleep_for(100ms);

        auto recv_queue = umt::BasicObjManager<serial::ReceiveQueue>::find_or_create("receive_queue");

        while (g_running) {
            serial::ReceiveQueue& queue = recv_queue->get();

            while (!queue.empty()) {
                serial::SerialReceiveData data = queue.front();
                queue.pop();

                // 欧拉角转四元数 (ZYX顺序)
                double yaw_rad = data.yaw * M_PI / 180.0;
                double pitch_rad = data.pitch * M_PI / 180.0;
                double roll_rad = data.roll * M_PI / 180.0;

                Eigen::Quaterniond q =
                    Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(pitch_rad, Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(roll_rad, Eigen::Vector3d::UnitX());

                {
                    std::lock_guard lock(g_imu_mutex);
                    g_q_imu = q;
                    g_imu_valid = true;
                }
            }

            std::this_thread::sleep_for(1ms);
        }
    } catch (const std::exception& e) {
        fmt::print(fmt::fg(fmt::color::red), "串口线程异常: {}\n", e.what());
    }

    fmt::print("串口线程退出\n");
}

// ============================================================================
// 棋盘格检测与PnP
// ============================================================================

// PnP结果
struct PnPResult {
    Eigen::Vector3d position;
    double reproj_error;  // 重投影误差 (像素)
};

std::optional<PnPResult> detect_chessboard_pnp(
    const cv::Mat& image,
    std::vector<cv::Point2f>& corners_out)
{
    cv::Size pattern_size(BOARD_WIDTH, BOARD_HEIGHT);

    // 检测角点
    bool found = cv::findChessboardCorners(
        image, pattern_size, corners_out,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK
    );

    if (!found) {
        return std::nullopt;
    }

    // 亚像素精化
    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image;
    }

    cv::cornerSubPix(gray, corners_out, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));

    // 生成3D物体点 (Z=0平面)
    std::vector<cv::Point3f> object_points;
    for (int i = 0; i < BOARD_HEIGHT; ++i) {
        for (int j = 0; j < BOARD_WIDTH; ++j) {
            object_points.emplace_back(j * SQUARE_SIZE, i * SQUARE_SIZE, 0.0f);
        }
    }

    // PnP求解
    const cv::Mat& camera_matrix = tf::get_camera_matrix();
    const cv::Mat& dist_coeffs = tf::get_distort_coeffs();

    cv::Mat rvec, tvec;
    bool success = cv::solvePnP(
        object_points, corners_out,
        camera_matrix, dist_coeffs,
        rvec, tvec,
        false,
        cv::SOLVEPNP_ITERATIVE
    );

    if (!success) {
        return std::nullopt;
    }

    // 计算重投影误差
    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs, projected_points);

    double total_error = 0;
    for (size_t i = 0; i < corners_out.size(); ++i) {
        double dx = corners_out[i].x - projected_points[i].x;
        double dy = corners_out[i].y - projected_points[i].y;
        total_error += std::sqrt(dx * dx + dy * dy);
    }
    double reproj_error = total_error / static_cast<double>(corners_out.size());

    // 返回结果
    PnPResult result;
    result.position = Eigen::Vector3d(
        tvec.at<double>(0),
        tvec.at<double>(1),
        tvec.at<double>(2)
    );
    result.reproj_error = reproj_error;

    return result;
}

// ============================================================================
// 保存/加载采样数据
// ============================================================================

void save_samples(const std::string& filename) {
    std::lock_guard lock(g_samples_mutex);

    std::ofstream file(filename);
    if (!file.is_open()) {
        fmt::print(fmt::fg(fmt::color::red), "无法打开文件: {}\n", filename);
        return;
    }

    file << g_samples.size() << "\n";
    for (const auto& s : g_samples) {
        file << s.frame_id << " "
             << s.q_imu.w() << " " << s.q_imu.x() << " " << s.q_imu.y() << " " << s.q_imu.z() << " "
             << s.p_camera.x() << " " << s.p_camera.y() << " " << s.p_camera.z() << "\n";
    }

    fmt::print(fmt::fg(fmt::color::green), "已保存 {} 个采样到 {}\n", g_samples.size(), filename);
}

void load_samples(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        fmt::print(fmt::fg(fmt::color::red), "无法打开文件: {}\n", filename);
        return;
    }

    std::lock_guard lock(g_samples_mutex);
    g_samples.clear();

    size_t count;
    file >> count;

    for (size_t i = 0; i < count; ++i) {
        extrinsic_calib::CalibSample s;
        double qw, qx, qy, qz;
        file >> s.frame_id >> qw >> qx >> qy >> qz
             >> s.p_camera.x() >> s.p_camera.y() >> s.p_camera.z();
        s.q_imu = Eigen::Quaterniond(qw, qx, qy, qz);
        g_samples.push_back(s);
    }

    fmt::print(fmt::fg(fmt::color::green), "已加载 {} 个采样\n", g_samples.size());
}

// ============================================================================
// 主程序
// ============================================================================

int main() {
    fmt::print(fmt::fg(fmt::color::gold),
        "==================================================================\n"
        "               相机外参自动标定工具\n"
        "==================================================================\n"
        "棋盘格: {}x{}, 格子大小: {}mm\n"
        "==================================================================\n"
        "操作:\n"
        "  SPACE - 拍照采样 (保持标定板静止, 转动云台)\n"
        "  'c'   - 开始标定 (网格搜索 + Ceres)\n"
        "  'g'   - 只用网格搜索\n"
        "  'r'   - 重置所有采样\n"
        "  'd'   - 删除最后一个采样\n"
        "  's'   - 保存采样到文件\n"
        "  'l'   - 从文件加载采样\n"
        "  'q'   - 退出\n"
        "==================================================================\n"
        "标定步骤:\n"
        "  1. 将棋盘格标定板固定在某处 (重要: 不要动!)\n"
        "  2. 转动云台到不同角度, 每个角度按 SPACE 拍照\n"
        "  3. 尽量覆盖: yaw ±30°, pitch ±15°\n"
        "  4. 采集 20-50 个样本后按 'c' 标定\n"
        "==================================================================\n\n",
        BOARD_WIDTH, BOARD_HEIGHT, static_cast<int>(SQUARE_SIZE * 1000)
    );

    // 初始化日志
    debug::init_session("test_extrinsic_calib");

#if HAS_CERES
    fmt::print(fmt::fg(fmt::color::green), "Ceres 已启用\n\n");
#else
    fmt::print(fmt::fg(fmt::color::yellow), "Ceres 未启用，只能使用网格搜索\n\n");
#endif

    // 加载配置
    auto config = static_param::parse_file("hardware.toml");

    // 启动运行时参数热重载线程 (读取aimer.toml中的offset)
    std::thread param_thread([]() {
        runtime_param::parameter_run("aimer.toml");
    });
    param_thread.detach();

    // 等待参数加载完成
    runtime_param::wait_for_param("ok");

    // 初始化TF模块 (加载相机内参和基础外参)
    if (!tf::init()) {
        fmt::print(fmt::fg(fmt::color::red), "TF模块初始化失败!\n");
        return 1;
    }

    // 加载基础外参
    auto base_extrinsic = extrinsic_calib::load_base_extrinsic();
    fmt::print("基础外参加载完成\n");

    // 启动串口线程 (从配置文件读取)
    std::thread imu_thread(imu_receiver_thread);
    std::this_thread::sleep_for(500ms);

    // 检查串口
    {
        std::lock_guard lock(g_imu_mutex);
        if (g_imu_valid) {
            fmt::print(fmt::fg(fmt::color::green), "IMU数据接收正常\n");
        } else {
            fmt::print(fmt::fg(fmt::color::yellow), "警告: 未收到IMU数据\n");
        }
    }

    // 加载相机配置并打开相机
    camera::CameraConfig cam_config;
    cam_config.use_camera_sn = static_param::get_param<bool>(config, "Camera", "use_camera_sn");
    cam_config.camera_sn = static_param::get_param<std::string>(config, "Camera", "camera_sn");
    cam_config.use_mfs_config = static_param::get_param<bool>(config, "Camera", "use_config_from_file");
    std::string mfs_filename = static_param::get_param<std::string>(config, "Camera", "config_file_path");
    cam_config.mfs_config_path = std::string(CONFIG_DIR) + "/" + mfs_filename;
    cam_config.use_runtime_config = static_param::get_param<bool>(config, "Camera", "use_camera_config");

    // 加载相机运行时参数
    auto param_table = static_param::get_param_table(config, "Camera.config");
    for (const auto& [key, value] : param_table) {
        std::visit([&](const auto& v) {
            using T = std::decay_t<decltype(v)>;
            if constexpr (!std::is_same_v<T, std::vector<int64_t>>) {
                cam_config.runtime_params.emplace_back(key, camera::CameraParam(v));
            }
        }, value);
    }

    camera::HikCam cam(cam_config);
    cam.open();
    fmt::print(fmt::fg(fmt::color::green), "相机已打开\n\n");

    int frame_id = 0;

    // 主循环
    while (g_running) {
        try {
            cv::Mat& img = cam.capture();
            if (img.empty()) continue;

            cv::Mat display = img.clone();
            frame_id++;

            // 检测棋盘格
            std::vector<cv::Point2f> corners;
            auto pnp_result = detect_chessboard_pnp(img, corners);

            // 绘制检测结果
            if (!corners.empty()) {
                cv::drawChessboardCorners(display, cv::Size(BOARD_WIDTH, BOARD_HEIGHT), corners, pnp_result.has_value());
            }

            // 重投影误差警告阈值
            constexpr double REPROJ_WARN_THRESHOLD = 1.0;  // 像素

            if (pnp_result.has_value()) {
                auto& res = pnp_result.value();
                auto& p = res.position;
                std::string text = fmt::format("Pos: ({:.3f}, {:.3f}, {:.3f}) m", p.x(), p.y(), p.z());
                cv::putText(display, text, cv::Point(10, 60),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);

                double distance = p.norm();
                cv::putText(display, fmt::format("Distance: {:.3f} m", distance), cv::Point(10, 90),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);

                // 显示重投影误差
                cv::Scalar reproj_color = (res.reproj_error < REPROJ_WARN_THRESHOLD)
                    ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
                cv::putText(display, fmt::format("Reproj Error: {:.3f} px", res.reproj_error),
                    cv::Point(10, 150), cv::FONT_HERSHEY_SIMPLEX, 0.6, reproj_color, 2);

                // 重投影误差过大时警告
                if (res.reproj_error >= REPROJ_WARN_THRESHOLD) {
                    cv::putText(display, "WARNING: High reproj error! Re-calibrate intrinsics!",
                        cv::Point(10, 180), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
                }
            }

            // 显示IMU状态
            {
                std::lock_guard lock(g_imu_mutex);
                if (g_imu_valid) {
                    // 四元数转欧拉角显示
                    auto euler = g_q_imu.toRotationMatrix().eulerAngles(2, 1, 0);  // ZYX
                    std::string imu_text = fmt::format("IMU: Y={:.1f} P={:.1f} R={:.1f}",
                        euler[0] * 180 / M_PI,
                        euler[1] * 180 / M_PI,
                        euler[2] * 180 / M_PI);
                    cv::putText(display, imu_text, cv::Point(10, 210),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);
                } else {
                    cv::putText(display, "IMU: No Data", cv::Point(10, 210),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
                }
            }

            // 显示采样数
            {
                std::lock_guard lock(g_samples_mutex);
                std::string samples_text = fmt::format("Samples: {} (need >= {})", g_samples.size(), MIN_SAMPLES);
                cv::Scalar color = g_samples.size() >= MIN_SAMPLES
                    ? cv::Scalar(0, 255, 0) : cv::Scalar(255, 255, 0);
                cv::putText(display, samples_text, cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2);
            }

            // 显示提示
            cv::putText(display, "SPACE=Sample, C=Calibrate, Q=Quit",
                cv::Point(10, display.rows - 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);

            cv::imshow("Extrinsic Calibration", display);

            // 处理键盘输入
            int key = cv::waitKey(1) & 0xFF;

            if (key == 'q' || key == 'Q' || key == 27) {
                g_running = false;

            } else if (key == ' ') {
                // 采样
                if (!pnp_result.has_value()) {
                    fmt::print(fmt::fg(fmt::color::red), "未检测到棋盘格!\n");
                    continue;
                }

                auto& res = pnp_result.value();

                // 重投影误差警告
                if (res.reproj_error >= 1.0) {
                    fmt::print(fmt::fg(fmt::color::yellow),
                        "警告: 重投影误差较大 ({:.2f}px)，建议重新标定相机内参!\n",
                        res.reproj_error);
                }

                Eigen::Quaterniond q;
                bool imu_ok = false;
                {
                    std::lock_guard lock(g_imu_mutex);
                    if (g_imu_valid) {
                        q = g_q_imu;
                        imu_ok = true;
                    }
                }

                if (!imu_ok) {
                    fmt::print(fmt::fg(fmt::color::red), "未收到IMU数据!\n");
                    continue;
                }

                {
                    std::lock_guard lock(g_samples_mutex);
                    g_samples.emplace_back(q, res.position, frame_id);
                    fmt::print(fmt::fg(fmt::color::green),
                        "[{}] 采样成功! 共 {} 个样本, reproj={:.2f}px\n",
                        frame_id, g_samples.size(), res.reproj_error);
                }

            } else if (key == 'r' || key == 'R') {
                std::lock_guard lock(g_samples_mutex);
                g_samples.clear();
                fmt::print("采样已重置\n");

            } else if (key == 'd' || key == 'D') {
                std::lock_guard lock(g_samples_mutex);
                if (!g_samples.empty()) {
                    g_samples.pop_back();
                    fmt::print("已删除最后一个采样, 剩余 {} 个\n", g_samples.size());
                }

            } else if (key == 's' || key == 'S') {
                save_samples("extrinsic_samples.txt");

            } else if (key == 'l' || key == 'L') {
                load_samples("extrinsic_samples.txt");

            } else if (key == 'c' || key == 'C') {
                // 开始标定
                std::vector<extrinsic_calib::CalibSample> samples_copy;
                {
                    std::lock_guard lock(g_samples_mutex);
                    samples_copy = g_samples;
                }

                if (samples_copy.size() < MIN_SAMPLES) {
                    fmt::print(fmt::fg(fmt::color::red),
                        "样本数不足! 当前 {} 个，需要 >= {} 个\n",
                        samples_copy.size(), MIN_SAMPLES);
                    continue;
                }

                fmt::print("\n\n开始标定...\n");

#if HAS_CERES
                auto result = extrinsic_calib::calibrate(samples_copy, base_extrinsic, true);
#else
                auto result = extrinsic_calib::two_stage_search(samples_copy, base_extrinsic, true);
#endif

                result.print();

                // 记录标定结果到日志
                debug::print("info", "ExtrinsicCalib", "========== 外参标定结果 ==========");
                debug::print("info", "ExtrinsicCalib", "状态: {}", result.success ? "成功" : "失败");
                debug::print("info", "ExtrinsicCalib", "初始标准差: {:.4f} m", result.initial_std);
                debug::print("info", "ExtrinsicCalib", "最终标准差: {:.4f} m", result.final_std);
                debug::print("info", "ExtrinsicCalib", "改进: {:.1f}%", (1.0 - result.final_std / result.initial_std) * 100);
                debug::print("info", "ExtrinsicCalib", "平移 (m): x={:.4f}, y={:.4f}, z={:.4f}",
                    result.params.offset_x, result.params.offset_y, result.params.offset_z);
                debug::print("info", "ExtrinsicCalib", "偏差 (deg): roll={:.2f}, pitch={:.2f}, yaw={:.2f}",
                    result.params.delta_roll * 180.0 / M_PI,
                    result.params.delta_pitch * 180.0 / M_PI,
                    result.params.delta_yaw * 180.0 / M_PI);

                if (result.success) {
                    fmt::print(fmt::fg(fmt::color::green),
                        "\n========== 请将以下配置写入文件 ==========\n\n");

                    fmt::print("config/aimer.toml [Transformer]:\n");
                    fmt::print("    camera_offset_x = {:.4f}\n", result.params.offset_x);
                    fmt::print("    camera_offset_y = {:.4f}\n", result.params.offset_y);
                    fmt::print("    camera_offset_z = {:.4f}\n", result.params.offset_z);

                    // 输出修正后的完整旋转矩阵
                    Eigen::Matrix3d R_final = result.get_final_R_camera2gimbal();
                    fmt::print("\nconfig/camera.yaml:\n");
                    fmt::print("R_camera2gimbal: [ {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f} ]\n",
                        R_final(0,0), R_final(0,1), R_final(0,2),
                        R_final(1,0), R_final(1,1), R_final(1,2),
                        R_final(2,0), R_final(2,1), R_final(2,2));

                    // 记录配置到日志
                    debug::print("info", "ExtrinsicCalib", "config/aimer.toml [Transformer]:");
                    debug::print("info", "ExtrinsicCalib", "    camera_offset_x = {:.4f}", result.params.offset_x);
                    debug::print("info", "ExtrinsicCalib", "    camera_offset_y = {:.4f}", result.params.offset_y);
                    debug::print("info", "ExtrinsicCalib", "    camera_offset_z = {:.4f}", result.params.offset_z);
                    debug::print("info", "ExtrinsicCalib", "config/camera.yaml:");
                    debug::print("info", "ExtrinsicCalib", "R_camera2gimbal: [ {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f} ]",
                        R_final(0,0), R_final(0,1), R_final(0,2),
                        R_final(1,0), R_final(1,1), R_final(1,2),
                        R_final(2,0), R_final(2,1), R_final(2,2));

                    // 角度偏差较大时提醒
                    double max_delta = std::max({
                        std::abs(result.params.delta_roll),
                        std::abs(result.params.delta_pitch),
                        std::abs(result.params.delta_yaw)
                    });
                    if (max_delta > 0.05) {  // > 3°
                        fmt::print(fmt::fg(fmt::color::yellow),
                            "\n注意: 姿态偏差较大 (roll={:.2f}°, pitch={:.2f}°, yaw={:.2f}°)\n"
                            "      请更新 camera.yaml 中的 R_camera2gimbal\n",
                            result.params.delta_roll * 180 / M_PI,
                            result.params.delta_pitch * 180 / M_PI,
                            result.params.delta_yaw * 180 / M_PI);
                        debug::print("warning", "ExtrinsicCalib", "姿态偏差较大: roll={:.2f}°, pitch={:.2f}°, yaw={:.2f}°",
                            result.params.delta_roll * 180 / M_PI,
                            result.params.delta_pitch * 180 / M_PI,
                            result.params.delta_yaw * 180 / M_PI);
                    }
                }
                fmt::print("\n");

            } else if (key == 'g' || key == 'G') {
                // 只用网格搜索
                std::vector<extrinsic_calib::CalibSample> samples_copy;
                {
                    std::lock_guard lock(g_samples_mutex);
                    samples_copy = g_samples;
                }

                if (samples_copy.size() < MIN_SAMPLES) {
                    fmt::print(fmt::fg(fmt::color::red),
                        "样本数不足! 当前 {} 个，需要 >= {} 个\n",
                        samples_copy.size(), MIN_SAMPLES);
                    continue;
                }

                fmt::print("\n\n开始网格搜索...\n");
                auto result = extrinsic_calib::two_stage_search(samples_copy, base_extrinsic, true);
                result.print();

                // 记录网格搜索结果到日志
                debug::print("info", "ExtrinsicCalib", "========== 网格搜索结果 ==========");
                debug::print("info", "ExtrinsicCalib", "状态: {}", result.success ? "成功" : "失败");
                debug::print("info", "ExtrinsicCalib", "初始标准差: {:.4f} m", result.initial_std);
                debug::print("info", "ExtrinsicCalib", "最终标准差: {:.4f} m", result.final_std);
                debug::print("info", "ExtrinsicCalib", "平移 (m): x={:.4f}, y={:.4f}, z={:.4f}",
                    result.params.offset_x, result.params.offset_y, result.params.offset_z);
                debug::print("info", "ExtrinsicCalib", "偏差 (deg): roll={:.2f}, pitch={:.2f}, yaw={:.2f}",
                    result.params.delta_roll * 180.0 / M_PI,
                    result.params.delta_pitch * 180.0 / M_PI,
                    result.params.delta_yaw * 180.0 / M_PI);
            }

        } catch (const std::exception& e) {
            fmt::print(fmt::fg(fmt::color::red), "异常: {}\n", e.what());
            std::this_thread::sleep_for(100ms);
        }
    }

    cv::destroyAllWindows();

    if (imu_thread.joinable()) {
        imu_thread.join();
    }

    fmt::print("\n程序退出\n");
    return 0;
}
