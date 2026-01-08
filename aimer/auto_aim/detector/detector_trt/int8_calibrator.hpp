/**
 * @file int8_calibrator.hpp
 * @brief TensorRT INT8 量化校准器
 *
 * 使用校准图片集确定每层激活值的动态范围，生成校准缓存文件。
 * 校准只需运行一次，之后直接加载缓存即可。
 *
 * 用法:
 *   1. 在 asset/int8_calib_data/ 放入 100-500 张代表性图片
 *   2. 配置 int8 = true，首次运行会自动校准
 *   3. 生成 asset/int8_calib.cache，后续直接加载
 */

#ifndef AIMER_AUTOAIM_DETECTOR_INT8_CALIBRATOR_HPP
#define AIMER_AUTOAIM_DETECTOR_INT8_CALIBRATOR_HPP

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <NvInfer.h>
#include <cuda_runtime.h>

#include "plugin/debug/logger.hpp"

namespace autoaim::detector {

namespace fs = std::filesystem;

/**
 * @brief INT8 熵校准器
 *
 * 实现 IInt8EntropyCalibrator2 接口，使用熵校准算法。
 * 相比 MinMax 校准，熵校准通常能获得更好的精度。
 */
class Int8EntropyCalibrator : public nvinfer1::IInt8EntropyCalibrator2 {
public:
    /**
     * @brief 构造函数
     * @param input_size 模型输入尺寸 (正方形边长)
     * @param calib_images_dir 校准图片目录
     * @param calib_cache_file 校准缓存文件路径
     */
    Int8EntropyCalibrator(
        int input_size,
        const std::string& calib_images_dir,
        const std::string& calib_cache_file)
        : input_size_(input_size),
          calib_cache_file_(calib_cache_file),
          current_idx_(0)
    {
        // 加载校准图片列表
        load_image_list(calib_images_dir);

        if (image_files_.empty()) {
            throw std::runtime_error(
                "INT8 calibration failed: No images found in " + calib_images_dir + "\n"
                "Please put 100-500 representative images (jpg/png) in the directory.\n"
                "You can extract frames from recorded videos using:\n"
                "  ffmpeg -i video.avi -vf \"select=not(mod(n\\,10))\" -frames:v 200 "
                "int8_calib_data/frame_%04d.jpg"
            );
        }

        debug::print("info", "INT8Calibrator",
            "Found {} calibration images in {}", image_files_.size(), calib_images_dir);

        // 分配 GPU 内存
        input_size_bytes_ = 3 * input_size * input_size * sizeof(float);
        cudaMalloc(&device_input_, input_size_bytes_);

        // 分配 CPU 内存用于预处理
        host_input_.resize(3 * input_size * input_size);
    }

    ~Int8EntropyCalibrator() override {
        if (device_input_) cudaFree(device_input_);
    }

    // 禁止拷贝
    Int8EntropyCalibrator(const Int8EntropyCalibrator&) = delete;
    Int8EntropyCalibrator& operator=(const Int8EntropyCalibrator&) = delete;

    /**
     * @brief 获取批次大小
     * @return 批次大小 (INT8 校准一般用 batch=1)
     */
    int getBatchSize() const noexcept override { return 1; }

    /**
     * @brief 获取下一批校准数据
     * @param bindings 绑定指针数组
     * @param names 绑定名称数组
     * @param nbBindings 绑定数量
     * @return true 如果还有数据，false 如果校准完成
     */
    bool getBatch(void* bindings[], const char* names[], int nbBindings) noexcept override {
        if (current_idx_ >= image_files_.size()) {
            return false;  // 校准完成
        }

        // 读取图片
        cv::Mat img = cv::imread(image_files_[current_idx_]);
        if (img.empty()) {
            debug::print("warning", "INT8Calibrator",
                "Failed to read image: {}", image_files_[current_idx_]);
            current_idx_++;
            return getBatch(bindings, names, nbBindings);  // 跳过，尝试下一张
        }

        // CPU 预处理: letterbox + BGR→RGB + normalize + HWC→CHW
        preprocess_cpu(img);

        // 拷贝到 GPU
        cudaMemcpy(device_input_, host_input_.data(),
                   input_size_bytes_, cudaMemcpyHostToDevice);

        bindings[0] = device_input_;
        current_idx_++;

        // 进度日志
        if (current_idx_ % 50 == 0 || current_idx_ == image_files_.size()) {
            debug::print("info", "INT8Calibrator",
                "Calibration progress: {}/{} ({:.1f}%)",
                current_idx_, image_files_.size(),
                100.0f * current_idx_ / image_files_.size());
        }

        return true;
    }

    /**
     * @brief 读取校准缓存
     * @param length [out] 缓存数据长度
     * @return 缓存数据指针，如果不存在返回 nullptr
     */
    const void* readCalibrationCache(size_t& length) noexcept override {
        calib_cache_.clear();
        std::ifstream input(calib_cache_file_, std::ios::binary);
        if (input.good()) {
            input.seekg(0, std::ios::end);
            length = input.tellg();
            input.seekg(0, std::ios::beg);
            calib_cache_.resize(length);
            input.read(calib_cache_.data(), length);
            debug::print("info", "INT8Calibrator",
                "Loaded calibration cache: {} ({} bytes)", calib_cache_file_, length);
            return calib_cache_.data();
        }
        debug::print("info", "INT8Calibrator",
            "No calibration cache found, will run calibration...");
        return nullptr;
    }

    /**
     * @brief 写入校准缓存
     * @param cache 缓存数据指针
     * @param length 缓存数据长度
     */
    void writeCalibrationCache(const void* cache, size_t length) noexcept override {
        std::ofstream output(calib_cache_file_, std::ios::binary);
        if (output.good()) {
            output.write(reinterpret_cast<const char*>(cache), length);
            debug::print("info", "INT8Calibrator",
                "Saved calibration cache: {} ({} bytes)", calib_cache_file_, length);
        } else {
            debug::print("error", "INT8Calibrator",
                "Failed to save calibration cache: {}", calib_cache_file_);
        }
    }

private:
    /**
     * @brief 加载目录中的图片文件列表
     */
    void load_image_list(const std::string& dir) {
        if (!fs::exists(dir)) {
            debug::print("warning", "INT8Calibrator",
                "Calibration directory does not exist: {}", dir);
            return;
        }

        for (const auto& entry : fs::directory_iterator(dir)) {
            if (!entry.is_regular_file()) continue;

            std::string ext = entry.path().extension().string();
            std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

            if (ext == ".jpg" || ext == ".jpeg" || ext == ".png" || ext == ".bmp") {
                image_files_.push_back(entry.path().string());
            }
        }

        // 排序保证一致性 (不同运行产生相同结果)
        std::sort(image_files_.begin(), image_files_.end());
    }

    /**
     * @brief CPU 预处理 (与推理时相同的处理流程)
     *
     * 步骤: letterbox resize → BGR→RGB → normalize → HWC→CHW
     */
    void preprocess_cpu(const cv::Mat& img) {
        // 1. 计算 letterbox 参数
        float scale = std::min(
            static_cast<float>(input_size_) / img.cols,
            static_cast<float>(input_size_) / img.rows
        );
        int new_w = static_cast<int>(img.cols * scale);
        int new_h = static_cast<int>(img.rows * scale);
        int pad_x = (input_size_ - new_w) / 2;
        int pad_y = (input_size_ - new_h) / 2;

        // 2. Resize
        cv::Mat resized;
        cv::resize(img, resized, cv::Size(new_w, new_h));

        // 3. 创建 letterbox 图像 (填充灰色 114，与 YOLO 训练一致)
        cv::Mat letterbox(input_size_, input_size_, CV_8UC3, cv::Scalar(114, 114, 114));
        resized.copyTo(letterbox(cv::Rect(pad_x, pad_y, new_w, new_h)));

        // 4. BGR → RGB, normalize (/255), HWC → CHW
        for (int c = 0; c < 3; ++c) {
            for (int h = 0; h < input_size_; ++h) {
                for (int w = 0; w < input_size_; ++w) {
                    // BGR → RGB: channel 0 is R (from B), channel 2 is B (from R)
                    int src_c = 2 - c;
                    float pixel = letterbox.at<cv::Vec3b>(h, w)[src_c];
                    host_input_[c * input_size_ * input_size_ + h * input_size_ + w] =
                        pixel / 255.0f;
                }
            }
        }
    }

    int input_size_;
    std::string calib_cache_file_;
    std::vector<std::string> image_files_;
    std::vector<char> calib_cache_;
    std::vector<float> host_input_;
    void* device_input_ = nullptr;
    size_t input_size_bytes_ = 0;
    size_t current_idx_;
};

}  // namespace autoaim::detector

#endif  // AIMER_AUTOAIM_DETECTOR_INT8_CALIBRATOR_HPP
