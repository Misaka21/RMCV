/*************************************************************************
	* Copyright (c) 2024, Misaka21
	* All rights reserved.
	*
	*
	* File name    :   hik_camera.cpp
	* Brief        :   海康相机的相关函数
	* Revision     :   V3.0
	* Author       :   Misaka21
	* Date         :   2024.04.10
	* Update       :   2023.10.16  V1.0    完成基本代码的编写
	*                  2023.10.18  V1.1    完成单线程的基本代码编写
	*                  2023.10.21  V2.0    完成相机双线程读写缓存
	*                  2023.11.10  V2.1    完成BayerRG转Mat的操作
	*                  2024.01.01  V2.2    完成相机错误代码的优化
	*                  2024.04.20  V3.0    完成产线上工业相机的硬触发和相关参数的设置
	*                  2024.04.28  V3.1    加入Gamma选择和超时时间
	*                  2024.04.30  V4.0    RM发布者版本的海康取流
	*                  2024.09.14  V4.1    优化代码结构，加入日志输出，添加错误处理
	*                  2024.11.10  V4.2    加入fps设置
	*                  2025.01.20  V4.3    修改参数显示的bug
	*                  2025.01.23  V4.4    加入相机垂直翻转
	*                  2025.01.25  V5.0    完善代码，将设置抽象
	*                  TODO：加入垂直翻转，水平翻转，相机参数输出，简化设置相机参数流程
	*************************************************************************/

// Source file corresponding header
#include "hik_camera.hpp"

// C system headers

// C++ system headers
#include <algorithm>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

// Third-party library headers
#include <toml++/toml.hpp>

// Project headers
#include "hik_log.hpp"
#include "plugin/debug/logger.hpp"

namespace camera {
    auto convert_to_cam_info = [](const std::vector<std::pair<std::string, Param> > &param_vec)
        -> std::vector<std::pair<std::string, CAM_INFO> > {
        std::vector<std::pair<std::string, CAM_INFO> > result;
        result.reserve(param_vec.size());
        for (const auto &[key, param_value]: param_vec) {
            std::visit([&](const auto &value) {
                using T = std::decay_t<decltype(value)>;
                if constexpr (!std::is_same_v<T, std::vector<int64_t> >) {
                    // 只有当类型不是 vector<int64_t> 时才添加
                    result.emplace_back(key, static_cast<CAM_INFO>(value));
                } else {
                    // 跳过 vector 类型，可以记录日志
                    // debug::print(debug::PrintMode::WARNING, "Skipping vector parameter: {}", key);
                }
            }, param_value);
        }
        return result;
    };

    HikCam::HikCam() {
        const auto param = toml::parse_file(CONFIG_DIR"/hardware.toml");
        this->_param_from_toml = convert_to_cam_info(static_param::get_param_table(param, "Camera.config"));
        this->_use_camera_sn = static_param::get_param<bool>(param, "Camera", "use_camera_sn");
        this->_camera_sn = static_param::get_param<std::string>(param, "Camera", "camera_sn");
        this->_use_config_from_file = static_param::get_param<bool>(param, "Camera", "use_config_from_file");
        this->_config_file_path = static_param::get_param<std::string>(param, "Camera", "config_file_path");
        this->_use_camera_config = static_param::get_param<bool>(param, "Camera", "use_camera_config");

        this->_config_file_path = std::string(CONFIG_DIR) + "/" + _config_file_path;
    }

    bool HikCam::print_device_info(MV_CC_DEVICE_INFO *pstMVDevInfo) {
        if (NULL == pstMVDevInfo) {
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

    void HikCam::open() {
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

        // 枚举设备
        _nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &stDeviceList);
        HIKCAM_FATAL(MV_CC_EnumDevices(MV_USB_DEVICE, &stDeviceList));

        if (stDeviceList.nDeviceNum == 0) {
            throw std::runtime_error("Find No Devices!");
        }

        // 打印所有设备信息
        for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++) {
            fmt::print("[device {}]:\n", i);
            MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
            if (NULL == pDeviceInfo) {
                throw std::runtime_error("The Pointer of pstMVDevInfo is NULL!");
            }
            print_device_info(pDeviceInfo);
        }

        auto find_device_by_sn = [&](const std::string &sn, MV_CC_DEVICE_INFO_LIST &deviceList,
                                     int &deviceIndex) -> bool {
            char device_sn[INFO_MAX_BUFFER_SIZE];

            if (sn.empty()) {
                debug::print("warning", "camera", "Camera SN is empty");
                return false;
            }

            for (size_t i = 0; i < deviceList.nDeviceNum; ++i) {
                if (deviceList.pDeviceInfo[i]->nTLayerType == MV_USB_DEVICE) {
                    memcpy(device_sn,
                           deviceList.pDeviceInfo[i]->SpecialInfo.stUsb3VInfo.chSerialNumber,
                           INFO_MAX_BUFFER_SIZE);
                } else if (deviceList.pDeviceInfo[i]->nTLayerType == MV_GIGE_DEVICE) {
                    memcpy(device_sn,
                           deviceList.pDeviceInfo[i]->SpecialInfo.stGigEInfo.chSerialNumber,
                           INFO_MAX_BUFFER_SIZE);
                } else {
                    continue;
                }

                device_sn[INFO_MAX_BUFFER_SIZE - 1] = '\0';

                if (std::strncmp(device_sn, sn.c_str(), INFO_MAX_BUFFER_SIZE) == 0) {
                    deviceIndex = i;
                    debug::print("info", "camera", "Found camera with SN:{}", device_sn);
                    return true;
                }
            }
            return false; // 未找到设备
        };

        bool camera_opened = false;
        int device_index_to_use = 0;

        // 如果配置使用 SN，尝试按 SN 查找并打开（最多3次）
        if (_use_camera_sn) {
            debug::print("info", "camera", "Attempting to find camera by SN:{}", _camera_sn);

            int sn_index = -1;
            bool found = false;

            // 尝试三次查找设备
            for (int attempt = 0; attempt < 3 && !found; ++attempt) {
                // 每次查找前重新枚举设备，确保设备列表是最新的
                _nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &stDeviceList);
                HIKCAM_FATAL(MV_CC_EnumDevices(MV_USB_DEVICE, &stDeviceList));

                if (stDeviceList.nDeviceNum == 0) {
                    debug::print("warning", "camera", "No devices found in attempt {}", attempt + 1);
                    std::this_thread::sleep_for(std::chrono::seconds(5));
                    continue;
                }

                found = find_device_by_sn(_camera_sn, stDeviceList, sn_index);

                if (!found) {
                    debug::print("warning", "camera", "Camera with SN {} not found in attempt {}",
                                 _camera_sn, attempt + 1);
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
            }

            // 如果找到设备，尝试打开
            if (found) {
                try {
                    // 确保句柄干净
                    if (_handle != NULL) {
                        MV_CC_DestroyHandle(_handle);
                        _handle = NULL;
                    }

                    HIKCAM_FATAL(MV_CC_CreateHandle(&_handle, stDeviceList.pDeviceInfo[sn_index]));
                    HIKCAM_FATAL(MV_CC_OpenDevice(_handle));
                    debug::print("info", "camera", "Successfully opened camera with SN: {}", _camera_sn);
                    device_index_to_use = sn_index;
                    camera_opened = true;
                } catch (const std::exception &e) {
                    debug::print("error", "camera", "Failed to open found camera: {}", e.what());
                    camera_opened = false;
                }
            } else {
                debug::print("warning", "camera",
                             "Camera with SN {} not found after 3 attempts, will use default camera\n",
                             _camera_sn);
            }
        }

        // 如果没有成功打开相机，使用默认的第一个相机
        if (!camera_opened) {
            fmt::print("Using default camera index: 0\n");

            // 确保句柄干净
            if (_handle != NULL) {
                MV_CC_DestroyHandle(_handle);
                _handle = NULL;
            }

            // 尝试打开默认相机
            try {
                HIKCAM_FATAL(MV_CC_CreateHandle(&_handle, stDeviceList.pDeviceInfo[0]));
                HIKCAM_FATAL(MV_CC_OpenDevice(_handle));
                device_index_to_use = 0;
                camera_opened = true;
            } catch (const std::exception &e) {
                throw std::runtime_error(fmt::format("Failed to open default camera: {}", e.what()));
            }
        }

        // 设置 GigE 设备的网络包大小
        if (stDeviceList.pDeviceInfo[device_index_to_use]->nTLayerType == MV_GIGE_DEVICE) {
            int nPacketSize = MV_CC_GetOptimalPacketSize(_handle);
            if (nPacketSize > 0) {
                _nRet = MV_CC_SetIntValue(_handle, "GevSCPSPacketSize", nPacketSize);
                HIKCAM_WARN(MV_CC_SetIntValue(_handle, "GevSCPSPacketSize", nPacketSize));
            } else {
                debug::print(debug::PrintMode::WARNING, "Camera", "Get Packet Size fail nRet [0x{:X}]", nPacketSize);
            }
        }
        if (_use_config_from_file)
            HIKCAM_WARN(MV_CC_FeatureLoad(this->_handle, this->_config_file_path.c_str()));


        if (_use_camera_config) {
            this->set_camera_info_batch();

            this->check_and_print();
        }


        // 开始取流
        HIKCAM_FATAL(MV_CC_StartGrabbing(_handle));
    }


    auto HikCam::capture() -> cv::Mat & {
        MV_FRAME_OUT stImageInfo = {0};
        const int maxRetries = 5;
        int numRetries = 0;
        while (numRetries < maxRetries) {
            _nRet = MV_CC_GetImageBuffer(_handle, &stImageInfo, 1000);
            if (_nRet == MV_OK) {
                unsigned char *_pDstData = static_cast<unsigned char *>(stImageInfo.pBufAddr);
                frame_id = static_cast<int>(stImageInfo.stFrameInfo.nFrameNum);
                cv::Mat rawData(
                    stImageInfo.stFrameInfo.nHeight,
                    stImageInfo.stFrameInfo.nWidth,
                    CV_8UC1,
                    _pDstData
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
        if (numRetries == maxRetries) {
            throw std::runtime_error(fmt::format("Get Image failed after {} retries, last error code: 0x{:x}",
                                                 maxRetries, _nRet));
        }
        return _srcImage;
    }


    template<typename T>
    auto HikCam::get_camera_param(std::string_view param_name)
        -> std::optional<T> {
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


    void HikCam::check_and_print() {
        const auto check_param = [](const auto &actual, const auto &expected, std::string_view name) {
            using ActualType = std::decay_t<decltype(actual)>;
            using ExpectedType = std::decay_t<decltype(expected)>;
            if constexpr (std::is_same_v<ActualType, std::string>) {
                const bool is_match = (actual == expected);
                if (is_match) {
                    fmt::print("{}: {}\n", name, actual);
                } else {
                    fmt::print(fg(fmt::color::red), "   {}: {} (Expected: {})\n",
                               name, actual, expected);
                }
                return is_match;
            } else if constexpr (std::is_same_v<ActualType, std::vector<int64_t> > &&
                                 std::is_same_v<ExpectedType, std::vector<int64_t> >) {
                // 处理向量类型比较
                const bool is_match = (actual.size() == expected.size() &&
                                       std::equal(actual.begin(), actual.end(), expected.begin()));
                if (is_match) {
                    fmt::print("{}: [vector matched, size={}]\n", name, actual.size());
                } else {
                    fmt::print(fg(fmt::color::red), "   {}: [vector mismatch] ", name);
                    fmt::print("actual=[");
                    for (size_t i = 0; i < actual.size(); ++i) {
                        if (i > 0) fmt::print(", ");
                        fmt::print("{}", actual[i]);
                    }
                    fmt::print("] expected=[");
                    for (size_t i = 0; i < expected.size(); ++i) {
                        if (i > 0) fmt::print(", ");
                        fmt::print("{}", expected[i]);
                    }
                    fmt::print("]\n");
                }
                return is_match;
            } else {
                // 数值类型比较
                const bool is_match = std::abs(static_cast<double>(actual) -
                                               static_cast<double>(expected)) < 0.1;
                if (is_match) {
                    fmt::print("{}: {}\n", name, actual);
                } else {
                    fmt::print(fg(fmt::color::red), "{}: {} (Expected: {})\n",
                               name, actual, expected);
                }
                return is_match;
            }
        };
        fmt::print(fmt::fg(fmt::color::purple), "======================\n");
        for (const auto &[name, expected_variant]: _param_from_toml) {
            std::visit([&](const auto &expected_value) {
                using T = std::decay_t<decltype(expected_value)>;
                if constexpr (std::is_same_v<T, std::vector<int64_t> >) {
                    fmt::print(fg(fmt::color::orange),
                               "   {}: Skipping vector parameter check (not supported by camera API)\n", name);
                } else if (auto actual_opt = get_camera_param<T>(name)) {
                    const auto &actual_value = *actual_opt;
                    check_param(actual_value, expected_value, name);
                } else {
                    fmt::print(fg(fmt::color::orange), "{}: Could not read from camera.\n", name);
                }
            }, expected_variant);
        }
        fmt::print(fmt::fg(fmt::color::purple), "======================\n");
    }

    void HikCam::set_camera_info_batch() {
        for (const auto &[key, value_variant]: _param_from_toml) {
            std::visit([this, &key](auto &&value) {
                using T = std::decay_t<decltype(value)>;

                // 1. 特殊处理：跳过没有对应设置函数的 vector 类型
                if constexpr (std::is_same_v<T, std::vector<int64_t> >) {
                    debug::print(debug::PrintMode::WARNING, "Camera", "Skipping batch set for vector parameter '{}'",
                                 key);
                    return;
                }

                this->set_camera_info(key, value);
            }, value_variant);
        }
    }

    HikCam::~HikCam() {
        if (_handle != NULL) {
            HIKCAM_ERROR(MV_CC_StopGrabbing(_handle));
            HIKCAM_ERROR(MV_CC_RegisterImageCallBackEx(_handle, NULL, NULL));
            HIKCAM_ERROR(MV_CC_CloseDevice(_handle));
            HIKCAM_ERROR(MV_CC_DestroyHandle(_handle));
            _handle = NULL;
        }
    }
} // namespace camera
