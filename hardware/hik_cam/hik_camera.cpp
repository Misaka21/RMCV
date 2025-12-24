/*************************************************************************
 * Copyright (c) 2024, Misaka21
 * All rights reserved.
 *
 *
 * File name    :   hik_camera.cpp
 * Brief        :   海康相机的相关函数
 * Revision     :   V5.1
 * Author       :   Misaka21
 * Date         :   2024.04.10
 * Update       :   2025.12.17  V5.1    重构：将TOML配置读取与相机类解耦
 *************************************************************************/

// Source file corresponding header
#include "hik_camera.hpp"

// C++ system headers
#include <algorithm>
#include <chrono>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

// Project headers
#include "hik_log.hpp"
#include "plugin/debug/logger.hpp"

namespace camera {

// ============================================================================
// Constructors
// ============================================================================

HikCam::HikCam(const CameraConfig& config) : _config(config) {
    debug::print(debug::PrintMode::DEBUG, "Camera", "HikCam created with config");
}

HikCam::HikCam() : _config() {
    debug::print(debug::PrintMode::DEBUG, "Camera", "HikCam created with default config");
}

// ============================================================================
// Public Methods
// ============================================================================

void HikCam::open() {
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    bool camera_opened = false;
    int device_index_to_use = 0;

    // 1. 枚举设备（带重试）
    bool devices_found = false;
    for (int attempt = 0; attempt < MAX_RETRY_ATTEMPTS && !devices_found; ++attempt) {
        debug::print(debug::PrintMode::INFO, "Camera", "Enumerating devices, attempt {}/{}",
            attempt + 1, MAX_RETRY_ATTEMPTS);
        devices_found = _enumerate_devices(stDeviceList);
        if (!devices_found) {
            debug::print(debug::PrintMode::WARNING, "Camera",
                "No devices found, retrying in {} seconds...", RETRY_DELAY_SECONDS);
            std::this_thread::sleep_for(std::chrono::seconds(RETRY_DELAY_SECONDS));
        }
    }

    if (!devices_found) {
        debug::print(debug::PrintMode::FATAL,
            "Camera","No devices found after {} attempts!", MAX_RETRY_ATTEMPTS);
        std::exit(1);

    }

    // 2. 尝试通过SN打开相机（如果配置了SN）
    if (_config.use_camera_sn) {
        camera_opened = _open_camera_by_sn(_config.camera_sn, stDeviceList, device_index_to_use);
    }

    // 3. 如果SN方式失败，使用默认的第一个相机
    if (!camera_opened) {
        camera_opened = _open_camera_by_index(0, stDeviceList);
        device_index_to_use = 0;
    }

    // 4. 配置GigE设备
    _configure_gige_device(stDeviceList.pDeviceInfo[device_index_to_use]);

    // 5. 加载相机配置
    _load_camera_config();

    // 6. 开始取流
    HIKCAM_FATAL(MV_CC_StartGrabbing(_handle));
}

auto HikCam::capture() -> cv::Mat& {
    MV_FRAME_OUT stImageInfo = {0};
    int numRetries = 0;

    while (numRetries < MAX_RETRY_ATTEMPTS) {
        _nRet = MV_CC_GetImageBuffer(_handle, &stImageInfo, 1000);
        if (_nRet == MV_OK) {
            unsigned char* pData = static_cast<unsigned char*>(stImageInfo.pBufAddr);
            frame_id = static_cast<int>(stImageInfo.stFrameInfo.nFrameNum);
            host_timestamp = stImageInfo.stFrameInfo.nHostTimeStamp;

            cv::Mat rawData(
                stImageInfo.stFrameInfo.nHeight,
                stImageInfo.stFrameInfo.nWidth,
                CV_8UC1,
                pData
            );

            if (PixelType_Gvsp_Mono8 == stImageInfo.stFrameInfo.enPixelType) {
                cv::cvtColor(rawData, _srcImage, cv::COLOR_GRAY2RGB);
            } else if (PixelType_Gvsp_BayerRG8 == stImageInfo.stFrameInfo.enPixelType) {
                cv::cvtColor(rawData, _srcImage, cv::COLOR_BayerRG2RGB);
            } else {
                debug::print(debug::PrintMode::ERROR, "Camera", "Unsupported pixel format");
            }

            HIKCAM_WARN(MV_CC_FreeImageBuffer(_handle, &stImageInfo));
            break;
        } else {
            HIKCAM_WARN(_nRet);
            numRetries++;
        }
    }

    if (numRetries == MAX_RETRY_ATTEMPTS) {
        throw std::runtime_error(fmt::format(
            "Get Image failed after {} retries, last error code: 0x{:x}",
            MAX_RETRY_ATTEMPTS, _nRet
        ));
    }

    return _srcImage;
}

HikCam::~HikCam() {
    if (_handle != nullptr) {
        HIKCAM_ERROR(MV_CC_StopGrabbing(_handle));
        HIKCAM_ERROR(MV_CC_RegisterImageCallBackEx(_handle, nullptr, nullptr));
        HIKCAM_ERROR(MV_CC_CloseDevice(_handle));
        HIKCAM_ERROR(MV_CC_DestroyHandle(_handle));
        _handle = nullptr;
    }
}

// ============================================================================
// Private Methods
// ============================================================================

bool HikCam::_print_device_info(MV_CC_DEVICE_INFO* pstMVDevInfo) {
    if (pstMVDevInfo == nullptr) {
        throw std::runtime_error("The Pointer of pstMVDevInfo is NULL!");
    }

    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE) {
        int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);
        fmt::print("CurrentIp: {}.{}.{}.{}\n", nIp1, nIp2, nIp3, nIp4);
        fmt::print("UserDefinedName: {}\n\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    } else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE) {
        fmt::print("UserDefinedName: {}\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
        fmt::print("Serial Number: {}\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
        fmt::print("Device Number: {}\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.nDeviceNumber);
    } else {
        fmt::print("Not support.\n");
    }
    return true;
}

bool HikCam::_enumerate_devices(MV_CC_DEVICE_INFO_LIST& deviceList) {
    memset(&deviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    _nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &deviceList);
    HIKCAM_FATAL(_nRet);

    if (deviceList.nDeviceNum == 0) {
        debug::print(debug::PrintMode::WARNING, "Camera", "No devices found");
        return false;
    }

    // 打印所有设备信息
    for (unsigned int i = 0; i < deviceList.nDeviceNum; i++) {
        fmt::print("[device {}]:\n", i);
        MV_CC_DEVICE_INFO* pDeviceInfo = deviceList.pDeviceInfo[i];
        if (pDeviceInfo == nullptr) {
            throw std::runtime_error("The Pointer of pstMVDevInfo is NULL!");
        }
        _print_device_info(pDeviceInfo);
    }
    return true;
}

bool HikCam::_find_device_by_sn(const std::string& sn, const MV_CC_DEVICE_INFO_LIST& deviceList, int& deviceIndex) {
    if (sn.empty()) {
        debug::print(debug::PrintMode::WARNING, "Camera", "Camera SN is empty");
        return false;
    }

    char device_sn[INFO_BUFFER_SIZE];

    for (size_t i = 0; i < deviceList.nDeviceNum; ++i) {
        if (deviceList.pDeviceInfo[i]->nTLayerType == MV_USB_DEVICE) {
            memcpy(device_sn, deviceList.pDeviceInfo[i]->SpecialInfo.stUsb3VInfo.chSerialNumber, INFO_BUFFER_SIZE);
        } else if (deviceList.pDeviceInfo[i]->nTLayerType == MV_GIGE_DEVICE) {
            memcpy(device_sn, deviceList.pDeviceInfo[i]->SpecialInfo.stGigEInfo.chSerialNumber, INFO_BUFFER_SIZE);
        } else {
            continue;
        }

        device_sn[INFO_BUFFER_SIZE - 1] = '\0';

        if (std::strncmp(device_sn, sn.c_str(), INFO_BUFFER_SIZE) == 0) {
            deviceIndex = static_cast<int>(i);
            debug::print(debug::PrintMode::INFO, "Camera", "Found camera with SN: {}", device_sn);
            return true;
        }
    }
    return false;
}

bool HikCam::_open_camera_by_sn(const std::string& sn, MV_CC_DEVICE_INFO_LIST& deviceList, int& deviceIndex) {
    debug::print(debug::PrintMode::INFO, "Camera", "Attempting to find camera by SN: {}", sn);

    int sn_index = -1;
    bool found = false;

    // 尝试多次查找设备
    for (int attempt = 0; attempt < MAX_RETRY_ATTEMPTS && !found; ++attempt) {
        // 每次查找前重新枚举设备，确保设备列表是最新的
        _enumerate_devices(deviceList);

        if (deviceList.nDeviceNum == 0) {
            debug::print(debug::PrintMode::WARNING, "Camera", "No devices found in attempt {}", attempt + 1);
            std::this_thread::sleep_for(std::chrono::seconds(RETRY_DELAY_SECONDS));
            continue;
        }

        found = _find_device_by_sn(sn, deviceList, sn_index);

        if (!found) {
            debug::print(debug::PrintMode::WARNING, "Camera", "Camera with SN {} not found in attempt {}", sn, attempt + 1);
        }
    }

    if (found && sn_index >= 0) {
        try {
            // 确保句柄干净
            if (_handle != nullptr) {
                MV_CC_DestroyHandle(_handle);
                _handle = nullptr;
            }

            HIKCAM_FATAL(MV_CC_CreateHandle(&_handle, deviceList.pDeviceInfo[sn_index]));
            HIKCAM_FATAL(MV_CC_OpenDevice(_handle));
            deviceIndex = sn_index;
            return true;
        } catch (const std::exception& e) {
            debug::print(debug::PrintMode::ERROR, "Camera", "Failed to open found camera: {}", e.what());
        }
    } else {
        debug::print(debug::PrintMode::WARNING, "Camera",
            "Camera with SN {} not found after {} attempts, will use default camera",
            sn, MAX_RETRY_ATTEMPTS);
    }

    return false;
}

bool HikCam::_open_camera_by_index(int deviceIndex, const MV_CC_DEVICE_INFO_LIST& deviceList) {
    if (deviceIndex < 0 || deviceIndex >= static_cast<int>(deviceList.nDeviceNum)) {
        throw std::runtime_error(fmt::format("Invalid device index: {}", deviceIndex));
    }

    fmt::print("Using default camera index: {}\n", deviceIndex);

    // 确保句柄干净
    if (_handle != nullptr) {
        MV_CC_DestroyHandle(_handle);
        _handle = nullptr;
    }

    try {
        HIKCAM_FATAL(MV_CC_CreateHandle(&_handle, deviceList.pDeviceInfo[deviceIndex]));
        HIKCAM_FATAL(MV_CC_OpenDevice(_handle));
        return true;
    } catch (const std::exception& e) {
        throw std::runtime_error(fmt::format("Failed to open camera at index {}: {}", deviceIndex, e.what()));
    }
}

void HikCam::_configure_gige_device(const MV_CC_DEVICE_INFO* deviceInfo) {
    if (deviceInfo->nTLayerType == MV_GIGE_DEVICE) {
        int nPacketSize = MV_CC_GetOptimalPacketSize(_handle);
        if (nPacketSize > 0) {
            _nRet = MV_CC_SetIntValue(_handle, "GevSCPSPacketSize", nPacketSize);
            HIKCAM_WARN(_nRet);
        } else {
            debug::print(debug::PrintMode::WARNING, "Camera", "Get Packet Size fail nRet [0x{:X}]", nPacketSize);
        }
    }
}

void HikCam::_load_camera_config() {
    // Load MFS config file
    if (_config.use_mfs_config && !_config.mfs_config_path.empty()) {
        debug::print(debug::PrintMode::INFO, "Camera", "Loading MFS config: {}", _config.mfs_config_path);
        HIKCAM_WARN(MV_CC_FeatureLoad(_handle, _config.mfs_config_path.c_str()));
    }

    // Apply runtime parameters
    if (_config.use_runtime_config) {
        _set_camera_info_batch();
        _check_and_print();
    }
}

void HikCam::_set_camera_info_batch() {
    for (const auto& [key, value_variant] : _config.runtime_params) {
        std::visit([this, &key](auto&& value) {
            this->set_camera_info(key, value);
        }, value_variant);
    }
}

template <typename T>
auto HikCam::_get_camera_param(std::string_view param_name) -> std::optional<T> {
    if constexpr (std::is_same_v<T, double>) {
        MVCC_FLOATVALUE value = {0};
        if (MV_CC_GetFloatValue(this->_handle, param_name.data(), &value) == MV_OK) {
            return value.fCurValue;
        }
    } else if constexpr (std::is_same_v<T, int64_t>) {
        MVCC_INTVALUE value = {0};
        if (MV_CC_GetIntValue(this->_handle, param_name.data(), &value) == MV_OK) {
            return value.nCurValue;
        }
    } else if constexpr (std::is_same_v<T, bool>) {
        bool value = false;
        if (MV_CC_GetBoolValue(this->_handle, param_name.data(), &value) == MV_OK) {
            return value;
        }
    }
    return std::nullopt;
}

void HikCam::_check_and_print() {
    const auto check_param = [](const auto& actual, const auto& expected, std::string_view name) {
        using ActualType = std::decay_t<decltype(actual)>;

        if constexpr (std::is_same_v<ActualType, std::string>) {
            const bool is_match = (actual == expected);
            if (is_match) {
                fmt::print("{}: {}\n", name, actual);
            } else {
                fmt::print(fg(fmt::color::red), "   {}: {} (Expected: {})\n", name, actual, expected);
            }
            return is_match;
        } else {
            // 数值类型比较
            const bool is_match = std::abs(static_cast<double>(actual) - static_cast<double>(expected)) < 0.1;
            if (is_match) {
                fmt::print("{}: {}\n", name, actual);
            } else {
                fmt::print(fg(fmt::color::red), "{}: {} (Expected: {})\n", name, actual, expected);
            }
            return is_match;
        }
    };

    fmt::print(fmt::fg(fmt::color::purple), "======================\n");

    for (const auto& [name, expected_variant] : _config.runtime_params) {
        std::visit([&](const auto& expected_value) {
            using T = std::decay_t<decltype(expected_value)>;
            if (auto actual_opt = _get_camera_param<T>(name)) {
                const auto& actual_value = *actual_opt;
                check_param(actual_value, expected_value, name);
            } else {
                fmt::print(fg(fmt::color::orange), "{}: Could not read from camera.\n", name);
            }
        }, expected_variant);
    }

    fmt::print(fmt::fg(fmt::color::purple), "======================\n");
}

}  // namespace camera
