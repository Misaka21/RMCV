/**
 * @file trt_init_mutex.hpp
 * @brief TensorRT 初始化互斥锁
 *
 * TRT builder (createInferBuilder / parseFromFile / buildSerializedNetwork) 不是线程安全的。
 * 多个 TRT 检测器并发构造时需要通过此锁串行化初始化。
 * 推理 (enqueueV2) 和缓存加载 (deserializeCudaEngine) 不受影响。
 */

#ifndef AIMER_COMMON_TRT_INIT_MUTEX_HPP
#define AIMER_COMMON_TRT_INIT_MUTEX_HPP

#include <mutex>

namespace aimer {

inline std::mutex& trt_init_mutex() {
    static std::mutex mtx;
    return mtx;
}

}  // namespace aimer

#endif  // AIMER_COMMON_TRT_INIT_MUTEX_HPP
