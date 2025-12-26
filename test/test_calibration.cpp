/**
 * @file test_calibration.cpp
 * @brief 海康相机标定工具
 */

#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <filesystem>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "hardware/hik_cam/hik_camera.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/param/static_config.hpp"

// ==================== 标定板参数 ====================
constexpr int BOARD_WIDTH = 11;
constexpr int BOARD_HEIGHT = 8;
constexpr float SQUARE_SIZE = 20.0f;  // mm

// 预览缩放比例 (加速检测)
constexpr double PREVIEW_SCALE = 0.5;

// ==================== 全局变量 ====================
std::vector<std::vector<cv::Point2f>> image_points;
std::vector<std::vector<cv::Point3f>> object_points;
std::vector<cv::Mat> captured_images;
cv::Size image_size;

camera::CameraConfig load_camera_config(const toml::table& config) {
    camera::CameraConfig cam_config;
    cam_config.use_camera_sn = static_param::get_param<bool>(config, "Camera", "use_camera_sn");
    cam_config.camera_sn = static_param::get_param<std::string>(config, "Camera", "camera_sn");
    cam_config.use_mfs_config = static_param::get_param<bool>(config, "Camera", "use_config_from_file");
    std::string mfs_filename = static_param::get_param<std::string>(config, "Camera", "config_file_path");
    cam_config.mfs_config_path = std::string(CONFIG_DIR) + "/" + mfs_filename;
    cam_config.use_runtime_config = static_param::get_param<bool>(config, "Camera", "use_camera_config");

    auto param_table = static_param::get_param_table(config, "Camera.config");
    for (const auto& [key, value] : param_table) {
        std::visit([&](const auto& v) {
            using T = std::decay_t<decltype(v)>;
            if constexpr (!std::is_same_v<T, std::vector<int64_t>>) {
                cam_config.runtime_params.emplace_back(key, camera::CameraParam(v));
            }
        }, value);
    }
    return cam_config;
}

std::vector<cv::Point3f> create_object_points() {
    std::vector<cv::Point3f> corners;
    for (int i = 0; i < BOARD_HEIGHT; ++i) {
        for (int j = 0; j < BOARD_WIDTH; ++j) {
            corners.emplace_back(j * SQUARE_SIZE, i * SQUARE_SIZE, 0.0f);
        }
    }
    return corners;
}

/**
 * @brief 快速检测 (缩小图像)
 */
bool detect_chessboard_fast(const cv::Mat& frame, std::vector<cv::Point2f>& corners) {
    cv::Mat small, gray;
    cv::resize(frame, small, cv::Size(), PREVIEW_SCALE, PREVIEW_SCALE);
    cv::cvtColor(small, gray, cv::COLOR_BGR2GRAY);

    cv::Size board_size(BOARD_WIDTH, BOARD_HEIGHT);
    bool found = cv::findChessboardCorners(gray, board_size, corners,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);

    if (found) {
        // 缩放回原始坐标
        for (auto& pt : corners) {
            pt.x /= PREVIEW_SCALE;
            pt.y /= PREVIEW_SCALE;
        }
    }
    return found;
}

/**
 * @brief 精确检测 (原始分辨率 + 亚像素)
 */
bool detect_chessboard_precise(const cv::Mat& frame, std::vector<cv::Point2f>& corners) {
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    cv::Size board_size(BOARD_WIDTH, BOARD_HEIGHT);
    bool found = cv::findChessboardCorners(gray, board_size, corners,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

    if (found) {
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));
    }
    return found;
}

bool calibrate_camera(cv::Mat& camera_matrix, cv::Mat& dist_coeffs,
    std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs, double& rms_error) {
    if (image_points.size() < 10) {
        std::cerr << "Error: 至少需要 10 张图像，当前 " << image_points.size() << " 张\n";
        return false;
    }
    std::cout << "\n开始标定，使用 " << image_points.size() << " 张图像...\n";

    camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);

    rms_error = cv::calibrateCamera(object_points, image_points, image_size,
        camera_matrix, dist_coeffs, rvecs, tvecs, cv::CALIB_FIX_K3);
    return true;
}

void print_calibration_result(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs, double rms_error) {
    std::cout << "\n==================== 标定结果 ====================\n";
    std::cout << "RMS 重投影误差: " << rms_error << " 像素\n";
    std::cout << "图像尺寸: " << image_size.width << " x " << image_size.height << "\n";

    std::cout << "\n# 可直接复制到 YAML 文件:\n";
    std::cout << std::setprecision(16);
    std::cout << "camera_matrix:\n";
    std::cout << "  [ " << camera_matrix.at<double>(0, 0) << ", 0, "
              << camera_matrix.at<double>(0, 2) << ", 0,\n";
    std::cout << "    " << camera_matrix.at<double>(1, 1) << ", "
              << camera_matrix.at<double>(1, 2) << ",\n";
    std::cout << "    0, 0, 1 ]\n";

    std::cout << "distort_coeffs:\n";
    std::cout << "  [ " << dist_coeffs.at<double>(0) << ", "
              << dist_coeffs.at<double>(1) << ", "
              << dist_coeffs.at<double>(2) << ", "
              << dist_coeffs.at<double>(3) << ", "
              << dist_coeffs.at<double>(4) << " ]\n";
    std::cout << "===================================================\n";
}

void save_captured_images(const std::string& dir) {
    std::filesystem::create_directories(dir);
    for (size_t i = 0; i < captured_images.size(); ++i) {
        cv::imwrite(dir + "/calib_" + std::to_string(i) + ".png", captured_images[i]);
    }
    std::cout << "已保存 " << captured_images.size() << " 张图像到: " << dir << "\n";
}

int main() {
    std::cout << "\n========== 海康相机标定工具 ==========\n";
    std::cout << "标定板: " << BOARD_WIDTH << "x" << BOARD_HEIGHT << " 角点\n";
    std::cout << "操作: 空格=拍照  c=标定  u=撤销  q=退出\n";
    std::cout << "======================================\n\n";

    debug::init_session("test_calibration");

    try {
        auto config = static_param::parse_file("hardware.toml");
        camera::CameraConfig cam_config = load_camera_config(config);
        camera::HikCam cam(cam_config);
        cam.open();

        std::cout << "相机已打开\n";

        cv::namedWindow("Calibration", cv::WINDOW_NORMAL);
        cv::resizeWindow("Calibration", 1280, 800);

        std::vector<cv::Point3f> obj_pts = create_object_points();
        std::vector<cv::Point2f> corners;
        bool found = false;

        while (true) {
            cv::Mat& frame = cam.capture();
            if (frame.empty()) continue;

            image_size = frame.size();

            // 快速检测 (缩小图像)
            found = detect_chessboard_fast(frame, corners);

            // 绘制预览
            cv::Mat display;
            cv::resize(frame, display, cv::Size(), 0.5, 0.5);  // 缩小显示
            
            if (found) {
                // 缩放角点坐标用于显示
                std::vector<cv::Point2f> display_corners;
                for (const auto& pt : corners) {
                    display_corners.emplace_back(pt.x * 0.5f, pt.y * 0.5f);
                }
                cv::drawChessboardCorners(display, cv::Size(BOARD_WIDTH, BOARD_HEIGHT), display_corners, true);
                cv::putText(display, "SPACE to capture", cv::Point(10, 50),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            } else {
                cv::putText(display, "No chessboard", cv::Point(10, 50),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
            }

            cv::putText(display, "Captured: " + std::to_string(image_points.size()),
                cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);

            cv::imshow("Calibration", display);

            int key = cv::waitKey(1) & 0xFF;

            if (key == 27 || key == 'q') break;

            if (key == ' ' && found) {
                std::cout << "正在精确检测..." << std::flush;
                
                // 精确检测 (全分辨率 + 亚像素)
                std::vector<cv::Point2f> precise_corners;
                if (detect_chessboard_precise(frame, precise_corners)) {
                    image_points.push_back(precise_corners);
                    object_points.push_back(obj_pts);
                    captured_images.push_back(frame.clone());
                    std::cout << " 已采集 " << image_points.size() << " 张\n";
                } else {
                    std::cout << " 失败，请重试\n";
                }
            }

            if (key == 'u' && !image_points.empty()) {
                image_points.pop_back();
                object_points.pop_back();
                captured_images.pop_back();
                std::cout << "撤销，剩余 " << image_points.size() << " 张\n";
            }

            if (key == 's') save_captured_images("calibration_images");

            if (key == 'c') {
                cv::Mat camera_matrix, dist_coeffs;
                std::vector<cv::Mat> rvecs, tvecs;
                double rms_error;

                if (calibrate_camera(camera_matrix, dist_coeffs, rvecs, tvecs, rms_error)) {
                    print_calibration_result(camera_matrix, dist_coeffs, rms_error);
                    save_captured_images("calibration_images");
                }
            }
        }

        cv::destroyAllWindows();

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return -1;
    }

    return 0;
}
