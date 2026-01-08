/**
 * @file cuda_preprocess.hpp
 * @brief CUDA 图像预处理接口
 *
 * 在 GPU 上完成所有预处理:
 *   letterbox resize + BGR→RGB + normalize + HWC→CHW
 *
 * 支持 FP32 和 FP16 两种输出格式
 */

#ifndef AIMER_AUTOAIM_DETECTOR_CUDA_PREPROCESS_HPP
#define AIMER_AUTOAIM_DETECTOR_CUDA_PREPROCESS_HPP

#include <cuda_runtime.h>
#include <cuda_fp16.h>
#include <cstdint>

// nvcc 对 C++17 嵌套命名空间支持有限，使用分开写法
namespace autoaim {
namespace detector {

/**
 * @brief GPU 预处理: letterbox + BGR→RGB + normalize + HWC→CHW (FP32 输出)
 *
 * 一个 kernel 完成所有预处理，输出可直接用于推理
 *
 * @param src_device    输入图像 (GPU, BGR, HWC, uint8)
 * @param dst_device    输出张量 (GPU, RGB, CHW, float32, normalized)
 * @param src_width     输入图像宽度
 * @param src_height    输入图像高度
 * @param dst_size      输出尺寸 (正方形, 如 640)
 * @param scale_out     [out] 缩放比例 (用于后处理坐标还原)
 * @param pad_x_out     [out] X 方向 padding (用于后处理坐标还原)
 * @param pad_y_out     [out] Y 方向 padding (用于后处理坐标还原)
 * @param stream        CUDA 流
 * @param use_bilinear  是否使用双线性插值 (默认 false, 用最近邻)
 */
void cuda_preprocess(
    const uint8_t* src_device,
    float* dst_device,
    int src_width, int src_height,
    int dst_size,
    float* scale_out = nullptr,
    int* pad_x_out = nullptr,
    int* pad_y_out = nullptr,
    cudaStream_t stream = nullptr,
    bool use_bilinear = false
);

/**
 * @brief GPU 预处理: letterbox + BGR→RGB + normalize + HWC→CHW (FP16 输出)
 *
 * 与 cuda_preprocess 相同，但输出 FP16 格式，用于 FP16 输入的模型
 *
 * @param src_device    输入图像 (GPU, BGR, HWC, uint8)
 * @param dst_device    输出张量 (GPU, RGB, CHW, float16, normalized)
 * @param src_width     输入图像宽度
 * @param src_height    输入图像高度
 * @param dst_size      输出尺寸 (正方形, 如 640)
 * @param scale_out     [out] 缩放比例 (用于后处理坐标还原)
 * @param pad_x_out     [out] X 方向 padding (用于后处理坐标还原)
 * @param pad_y_out     [out] Y 方向 padding (用于后处理坐标还原)
 * @param stream        CUDA 流
 * @param use_bilinear  是否使用双线性插值 (默认 false, 用最近邻)
 */
void cuda_preprocess_fp16(
    const uint8_t* src_device,
    __half* dst_device,
    int src_width, int src_height,
    int dst_size,
    float* scale_out = nullptr,
    int* pad_x_out = nullptr,
    int* pad_y_out = nullptr,
    cudaStream_t stream = nullptr,
    bool use_bilinear = false
);

}  // namespace detector
}  // namespace autoaim

#endif  // AIMER_AUTOAIM_DETECTOR_CUDA_PREPROCESS_HPP
