/**
 * @file cuda_preprocess.cu
 * @brief CUDA 图像预处理实现
 *
 * 一个 kernel 完成所有预处理:
 *   letterbox resize + BGR→RGB + normalize + HWC→CHW
 */

#include "cuda_preprocess.hpp"
#include <algorithm>

// nvcc 对 C++17 嵌套命名空间支持有限，使用分开写法
namespace autoaim {
namespace detector {

/**
 * @brief Letterbox + BGR→RGB + Normalize + HWC→CHW kernel
 *
 * 每个线程处理输出张量的一个位置 (c, y, x)
 */
__global__ void preprocess_kernel(
    const uint8_t* __restrict__ src,  // 输入: BGR, HWC, uint8
    float* __restrict__ dst,           // 输出: RGB, CHW, float32
    int src_width, int src_height,     // 原图尺寸
    int dst_size,                      // 目标尺寸 (正方形)
    int resized_w, int resized_h,      // resize 后尺寸
    int pad_x, int pad_y,              // padding 偏移
    float scale                        // 缩放比例
) {
    // 输出位置
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int c = blockIdx.z;  // 0=R, 1=G, 2=B

    if (x >= dst_size || y >= dst_size || c >= 3) return;

    float pixel_value;

    // 检查是否在有效区域内
    int src_x_in_resized = x - pad_x;
    int src_y_in_resized = y - pad_y;

    if (src_x_in_resized < 0 || src_x_in_resized >= resized_w ||
        src_y_in_resized < 0 || src_y_in_resized >= resized_h) {
        // padding 区域，填充灰色 (114/255)
        pixel_value = 114.0f / 255.0f;
    } else {
        // 计算源图像坐标 (双线性插值)
        float src_x = src_x_in_resized / scale;
        float src_y = src_y_in_resized / scale;

        // 最近邻采样 (简单快速)
        int ix = min(max(int(src_x + 0.5f), 0), src_width - 1);
        int iy = min(max(int(src_y + 0.5f), 0), src_height - 1);

        // BGR → RGB: c=0 对应 B(2), c=1 对应 G(1), c=2 对应 R(0)
        int bgr_c = 2 - c;

        // 读取像素并归一化
        int src_idx = (iy * src_width + ix) * 3 + bgr_c;
        pixel_value = src[src_idx] / 255.0f;
    }

    // 输出 CHW 格式: dst[c][y][x]
    int dst_idx = c * dst_size * dst_size + y * dst_size + x;
    dst[dst_idx] = pixel_value;
}

/**
 * @brief 双线性插值版本 (更高质量)
 */
__global__ void preprocess_kernel_bilinear(
    const uint8_t* __restrict__ src,
    float* __restrict__ dst,
    int src_width, int src_height,
    int dst_size,
    int resized_w, int resized_h,
    int pad_x, int pad_y,
    float scale
) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int c = blockIdx.z;

    if (x >= dst_size || y >= dst_size || c >= 3) return;

    float pixel_value;

    int src_x_in_resized = x - pad_x;
    int src_y_in_resized = y - pad_y;

    if (src_x_in_resized < 0 || src_x_in_resized >= resized_w ||
        src_y_in_resized < 0 || src_y_in_resized >= resized_h) {
        pixel_value = 114.0f / 255.0f;
    } else {
        // 双线性插值
        float src_x = src_x_in_resized / scale;
        float src_y = src_y_in_resized / scale;

        int x0 = int(src_x);
        int y0 = int(src_y);
        int x1 = min(x0 + 1, src_width - 1);
        int y1 = min(y0 + 1, src_height - 1);
        x0 = max(x0, 0);
        y0 = max(y0, 0);

        float dx = src_x - x0;
        float dy = src_y - y0;

        int bgr_c = 2 - c;

        float v00 = src[(y0 * src_width + x0) * 3 + bgr_c];
        float v01 = src[(y0 * src_width + x1) * 3 + bgr_c];
        float v10 = src[(y1 * src_width + x0) * 3 + bgr_c];
        float v11 = src[(y1 * src_width + x1) * 3 + bgr_c];

        float v0 = v00 * (1 - dx) + v01 * dx;
        float v1 = v10 * (1 - dx) + v11 * dx;
        pixel_value = (v0 * (1 - dy) + v1 * dy) / 255.0f;
    }

    int dst_idx = c * dst_size * dst_size + y * dst_size + x;
    dst[dst_idx] = pixel_value;
}

// ============================================================================
// 公开接口
// ============================================================================

void cuda_preprocess(
    const uint8_t* src_device,
    float* dst_device,
    int src_width, int src_height,
    int dst_size,
    float* scale_out,
    int* pad_x_out, int* pad_y_out,
    cudaStream_t stream,
    bool use_bilinear
) {
    // 计算 letterbox 参数
    float scale = std::min(
        static_cast<float>(dst_size) / src_width,
        static_cast<float>(dst_size) / src_height
    );

    int resized_w = static_cast<int>(src_width * scale);
    int resized_h = static_cast<int>(src_height * scale);
    int pad_x = (dst_size - resized_w) / 2;
    int pad_y = (dst_size - resized_h) / 2;

    // 返回参数 (用于后处理坐标还原)
    if (scale_out) *scale_out = scale;
    if (pad_x_out) *pad_x_out = pad_x;
    if (pad_y_out) *pad_y_out = pad_y;

    // 启动 kernel
    dim3 block(16, 16, 1);
    dim3 grid(
        (dst_size + block.x - 1) / block.x,
        (dst_size + block.y - 1) / block.y,
        3  // RGB 三通道
    );

    if (use_bilinear) {
        preprocess_kernel_bilinear<<<grid, block, 0, stream>>>(
            src_device, dst_device,
            src_width, src_height,
            dst_size, resized_w, resized_h,
            pad_x, pad_y, scale
        );
    } else {
        preprocess_kernel<<<grid, block, 0, stream>>>(
            src_device, dst_device,
            src_width, src_height,
            dst_size, resized_w, resized_h,
            pad_x, pad_y, scale
        );
    }
}

// ============================================================================
// FP16 版本
// ============================================================================

/**
 * @brief FP16 版本: Letterbox + BGR→RGB + Normalize + HWC→CHW kernel
 */
__global__ void preprocess_kernel_fp16(
    const uint8_t* __restrict__ src,
    __half* __restrict__ dst,
    int src_width, int src_height,
    int dst_size,
    int resized_w, int resized_h,
    int pad_x, int pad_y,
    float scale
) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int c = blockIdx.z;

    if (x >= dst_size || y >= dst_size || c >= 3) return;

    float pixel_value;

    int src_x_in_resized = x - pad_x;
    int src_y_in_resized = y - pad_y;

    if (src_x_in_resized < 0 || src_x_in_resized >= resized_w ||
        src_y_in_resized < 0 || src_y_in_resized >= resized_h) {
        pixel_value = 114.0f / 255.0f;
    } else {
        float src_x = src_x_in_resized / scale;
        float src_y = src_y_in_resized / scale;

        int ix = min(max(int(src_x + 0.5f), 0), src_width - 1);
        int iy = min(max(int(src_y + 0.5f), 0), src_height - 1);

        int bgr_c = 2 - c;
        int src_idx = (iy * src_width + ix) * 3 + bgr_c;
        pixel_value = src[src_idx] / 255.0f;
    }

    int dst_idx = c * dst_size * dst_size + y * dst_size + x;
    dst[dst_idx] = __float2half(pixel_value);
}

/**
 * @brief FP16 双线性插值版本
 */
__global__ void preprocess_kernel_bilinear_fp16(
    const uint8_t* __restrict__ src,
    __half* __restrict__ dst,
    int src_width, int src_height,
    int dst_size,
    int resized_w, int resized_h,
    int pad_x, int pad_y,
    float scale
) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int c = blockIdx.z;

    if (x >= dst_size || y >= dst_size || c >= 3) return;

    float pixel_value;

    int src_x_in_resized = x - pad_x;
    int src_y_in_resized = y - pad_y;

    if (src_x_in_resized < 0 || src_x_in_resized >= resized_w ||
        src_y_in_resized < 0 || src_y_in_resized >= resized_h) {
        pixel_value = 114.0f / 255.0f;
    } else {
        float src_x = src_x_in_resized / scale;
        float src_y = src_y_in_resized / scale;

        int x0 = int(src_x);
        int y0 = int(src_y);
        int x1 = min(x0 + 1, src_width - 1);
        int y1 = min(y0 + 1, src_height - 1);
        x0 = max(x0, 0);
        y0 = max(y0, 0);

        float dx = src_x - x0;
        float dy = src_y - y0;

        int bgr_c = 2 - c;

        float v00 = src[(y0 * src_width + x0) * 3 + bgr_c];
        float v01 = src[(y0 * src_width + x1) * 3 + bgr_c];
        float v10 = src[(y1 * src_width + x0) * 3 + bgr_c];
        float v11 = src[(y1 * src_width + x1) * 3 + bgr_c];

        float v0 = v00 * (1 - dx) + v01 * dx;
        float v1 = v10 * (1 - dx) + v11 * dx;
        pixel_value = (v0 * (1 - dy) + v1 * dy) / 255.0f;
    }

    int dst_idx = c * dst_size * dst_size + y * dst_size + x;
    dst[dst_idx] = __float2half(pixel_value);
}

void cuda_preprocess_fp16(
    const uint8_t* src_device,
    __half* dst_device,
    int src_width, int src_height,
    int dst_size,
    float* scale_out,
    int* pad_x_out, int* pad_y_out,
    cudaStream_t stream,
    bool use_bilinear
) {
    // 计算 letterbox 参数
    float scale = std::min(
        static_cast<float>(dst_size) / src_width,
        static_cast<float>(dst_size) / src_height
    );

    int resized_w = static_cast<int>(src_width * scale);
    int resized_h = static_cast<int>(src_height * scale);
    int pad_x = (dst_size - resized_w) / 2;
    int pad_y = (dst_size - resized_h) / 2;

    if (scale_out) *scale_out = scale;
    if (pad_x_out) *pad_x_out = pad_x;
    if (pad_y_out) *pad_y_out = pad_y;

    dim3 block(16, 16, 1);
    dim3 grid(
        (dst_size + block.x - 1) / block.x,
        (dst_size + block.y - 1) / block.y,
        3
    );

    if (use_bilinear) {
        preprocess_kernel_bilinear_fp16<<<grid, block, 0, stream>>>(
            src_device, dst_device,
            src_width, src_height,
            dst_size, resized_w, resized_h,
            pad_x, pad_y, scale
        );
    } else {
        preprocess_kernel_fp16<<<grid, block, 0, stream>>>(
            src_device, dst_device,
            src_width, src_height,
            dst_size, resized_w, resized_h,
            pad_x, pad_y, scale
        );
    }
}

}  // namespace detector
}  // namespace autoaim
