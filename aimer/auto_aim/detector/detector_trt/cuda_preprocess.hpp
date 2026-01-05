/**
 * @file cuda_preprocess.hpp
 * @brief CUDA 图像预处理接口
 */

#ifndef AIMER_AUTOAIM_DETECTOR_CUDA_PREPROCESS_HPP
#define AIMER_AUTOAIM_DETECTOR_CUDA_PREPROCESS_HPP

#include <cuda_runtime.h>
#include <cstdint>

namespace autoaim::detector {

/**
 * @brief GPU 预处理: letterbox + BGR→RGB + normalize + HWC→CHW
 *
 * @param src_device 输入图像 (GPU, BGR, HWC, uint8)
 * @param dst_device 输出张量 (GPU, RGB, CHW, float32, normalized)
 * @param src_width 输入宽度
 * @param src_height 输入高度
 * @param dst_size 输出尺寸 (正方形)
 * @param stream CUDA 流
 */
void cuda_preprocess(
    const uint8_t* src_device,
    float* dst_device,
    int src_width, int src_height,
    int dst_size,
    cudaStream_t stream
);

/**
 * @brief GPU 预处理 (返回 letterbox 参数用于后处理坐标还原)
 */
void cuda_preprocess_with_params(
    const uint8_t* src_device,
    float* dst_device,
    int src_width, int src_height,
    int dst_size,
    float* out_scale,
    int* out_dx,
    int* out_dy,
    cudaStream_t stream
);

}  // namespace autoaim::detector

#endif  // AIMER_AUTOAIM_DETECTOR_CUDA_PREPROCESS_HPP
