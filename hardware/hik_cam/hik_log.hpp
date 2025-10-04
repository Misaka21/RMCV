//
// Created by nuc11 on 2025/10/3.
//
#ifndef RMCV_HIK_LOG_HPP
#define RMCV_HIK_LOG_HPP

#include "plugin/debug/logger.hpp"
// 注意：使用以下宏前，请确保已定义变量 `_nRet`，例如: int _nRet;
// 警告处理宏定义
#define HIKCAM_WARN(func) \
do { \
    _nRet = func; \
    if (_nRet != MV_OK) { \
        debug::print(debug::PrintMode::WARNING, \
                    #func, \
                    " failed!, error code: 0x{:x}", \
                    static_cast<unsigned>(_nRet)); \
    } \
} while(0)

// 错误处理宏定义
#define HIKCAM_ERROR(func) \
do { \
    _nRet = func; \
    if (_nRet != MV_OK) { \
        debug::print(debug::PrintMode::ERROR, \
                    #func, \
                    " failed!, error code: 0x{:x}", \
                    static_cast<unsigned>(_nRet)); \
    } \
} while(0)
// 致命错误处理宏定义
#define HIKCAM_FATAL(func) \
do { \
    _nRet = func; \
    if (_nRet != MV_OK) { \
        throw std::runtime_error(           \
            fmt::format("in {}: {} failed!, error code: 0x{:x}",\
            __LINE__, #func, static_cast<unsigned>(_nRet))); \
    } \
} while(0)
#endif //RMCV_HIK_LOG_HPP