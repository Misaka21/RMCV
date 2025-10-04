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
 *                 2025.01.25  V5.0    完善代码，将设置抽象
*                  TODO：加入垂直翻转，水平翻转，相机参数输出，简化设置相机参数流程
*************************************************************************/
#include "hik_camera.hpp"
#include "plugin/debug/logger.hpp"
#include <optional>

namespace camera {

bool HikCam::PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo) {
   if (NULL == pstMVDevInfo) {
       printf("The Pointer of pstMVDevInfo is NULL!\n");
       return false;
   }
   if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE) {
       int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
       int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
       int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
       int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

       // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined name
       printf("CurrentIp: %d.%d.%d.%d\n", nIp1, nIp2, nIp3, nIp4);
       printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
   } else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE) {
       printf("UserDefinedName: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
       printf("Serial Number: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
       printf("Device Number: %d\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.nDeviceNumber);
   } else {
       printf("Not support.\n");
   }


   return true;
}

void HikCam::open()() {
   // ch:初始化SDK | en:Initialize SDK
   //有的版本不适用，故注释
   // _nRet = MV_CC_Initialize();
   // if (MV_OK != _nRet)
   // {
   //    //printf("Initialize SDK fail! nRet [0x%x]\n", _nRet);
   //    std::cout<<RED_START<<"[ERROR]: Initialize SDK fail! nRet [0x%x]\n"<<COLOR_END<<_nRet<<std::endl;
   //    //break;
   // }

   // ch:枚举设备 | Enum device
   MV_CC_DEVICE_INFO_LIST stDeviceList;
   memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
   //_nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
   //加快开机时间
   _nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &stDeviceList);
   if (MV_OK != _nRet) {
       throw std::runtime_error("Enum Devices fail! nRet [0x" + std::to_string(_nRet) + "]");
       //std::cout<<RED_START<<"[ERROR]Enum Devices fail! nRet [0x%x]\n"<<COLOR_END<<_nRet<<std::endl;
       //printf("Enum Devices fail! nRet [0x%x]\n", _nRet);
       //break;
   }

   if (stDeviceList.nDeviceNum > 0) {
       for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++) {
           printf("[device %d]:\n", i);
           MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
           if (NULL == pDeviceInfo) {
               throw std::runtime_error("The Pointer of pstMVDevInfo is NULL!");
               //break;
           }
           PrintDeviceInfo(pDeviceInfo);
       }
   } else {

       throw std::runtime_error("Find No Devices!");
       //std::cout<<RED_START<<"[ERROR]: Find No Devices!\n"<<COLOR_END<<std::endl;
       //printf("Find No Devices!\n");
       //break;
   }

   printf("Please Input camera index(0-%d):", stDeviceList.nDeviceNum - 1);
   int nIndex = Info.parameters.count("CamID")?std::get<int>(Info.parameters.at("CamID")) : 0;
   printf(" %d\n", nIndex);
   //Info.parameters.erase<int>("CamID");

   // ch:选择设备并创建句柄 | Select device and create handle
   _nRet = MV_CC_CreateHandle(&_handle, stDeviceList.pDeviceInfo[nIndex]);
   if (MV_OK != _nRet) {
       throw std::runtime_error("Create handle fail! nRet [0x" + std::to_string(_nRet) + "]");
       //printf("Create _handle fail! nRet [0x%x]\n", _nRet);
       //break;
   }

   // ch:打开设备 | Open device
   _nRet = MV_CC_OpenDevice(_handle);
   if (MV_OK != _nRet) {
       throw std::runtime_error(fmt::format("Open Device fail! nRet [0x{:x}]", _nRet));
       //throw std::runtime_error("Open Device fail! nRet [0x" + std::to_string(_nRet) + "]");
       //printf("%sOpen Device fail! nRet [0x%x]%s\n", RED_START, _nRet, COLOR_END);
       //printf("Open Device fail! nRet [0x%x]\n", _nRet);
       //break;
   }

   // ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
   if (stDeviceList.pDeviceInfo[nIndex]->nTLayerType == MV_GIGE_DEVICE) {
       int nPacketSize = MV_CC_GetOptimalPacketSize(_handle);
       if (nPacketSize > 0) {
           _nRet = MV_CC_SetIntValue(_handle, "GevSCPSPacketSize", nPacketSize);
           if (_nRet != MV_OK) {
               debug::print(
                   debug::PrintMode::WARNING,
                   "Camera",
                   "Set Packet Size fail nRet [0x{:X}]",
                   _nRet
               );
               //printf("%s[Warning]: Set Packet Size fail nRet [0x%x]!%s\n", YELLOW_START, _nRet, COLOR_END);
               //printf("Warning: Set Packet Size fail nRet [0x%x]!", _nRet);
           }
       } else {
           debug::print(
               debug::PrintMode::WARNING,
               "Camera",
               "Get Packet Size fail nRet [0x{:X}]",
               nPacketSize
           );
           //printf("%s[Warning]: Get Packet Size fail nRet [0x%x]!%s\n", YELLOW_START, nPacketSize, COLOR_END);
           //printf("Warning: Get Packet Size fail nRet [0x%x]!", nPacketSize);
       }
   }

   // ch:设置触发模式为off | eb:Set trigger mode as off
   _nRet = MV_CC_SetEnumValue(_handle, "TriggerMode", MV_TRIGGER_MODE_OFF);
   if (MV_OK != _nRet) {
       throw std::runtime_error("Set Trigger Mode fail! nRet [0x" + std::to_string(_nRet) + "]");
       //printf("%s[ERROR]: Set Trigger Mode fail! nRet [0x%x]%s\n", RED_START, _nRet, COLOR_END);
       //printf("Set Trigger Mode fail! nRet [0x%x]\n", _nRet);
       //break;
   }
   SetAttribute(Info);

   // ch:开始取流 | en:Start grab image
   _nRet = MV_CC_StartGrabbing(_handle);
   if (MV_OK != _nRet) {
       throw std::runtime_error("Start Grabbing fail! nRet [0x" + std::to_string(_nRet) + "]");
       //printf("%s[ERROR]: Start Grabbing fail! nRet [0x%x]%s\n", RED_START, _nRet, COLOR_END);
       //printf("Start Grabbing fail! nRet [0x%x]\n", _nRet);
       //break;
   }
}

auto HikCam::capture() -> cv::Mat& {
   MV_FRAME_OUT stImageInfo = { 0 };
   const int maxRetries = 5; // 最大重试次数
   int numRetries = 0;

   while (numRetries < maxRetries) {
       _nRet = MV_CC_GetImageBuffer(_handle, &stImageInfo, 1000);

       if (_nRet == MV_OK) {
           unsigned char* _pDstData = static_cast<unsigned char*>(stImageInfo.pBufAddr); // 直接使用提供的缓冲区

           frame_id = static_cast<int>(stImageInfo.stFrameInfo.nFrameNum);
           
           // 直接在原始数据上创建Mat对象,避免内存拷贝
           cv::Mat rawData(
               stImageInfo.stFrameInfo.nHeight,
               stImageInfo.stFrameInfo.nWidth,
               CV_8UC1,
               _pDstData
           );

           if (PixelType_Gvsp_Mono8 == stImageInfo.stFrameInfo.enPixelType) {
               // 直接转换到目标图像
               cv::cvtColor(rawData, _srcImage, cv::COLOR_GRAY2RGB);
           } else if (PixelType_Gvsp_BayerRG8 == stImageInfo.stFrameInfo.enPixelType) {
               // 直接转换到目标图像
               cv::cvtColor(rawData, _srcImage, cv::COLOR_BayerRG2RGB);
           } else {
               debug::print(debug::PrintMode::ERROR, "Camera", "Unsupported pixel format");
           }

           _nRet = MV_CC_FreeImageBuffer(_handle, &stImageInfo);
           if (_nRet != MV_OK) {
               debug::print(
                   debug::PrintMode::ERROR,
                   "Camera",
                   "Free Image Buffer fail! nRet [0x{:X}]",
                   _nRet
               );
           }
           break;
       } else {
           debug::print(
               debug::PrintMode::WARNING,
               "Camera",
               "Get Image fail! nRet [0x{:X}]",
               _nRet
           );
           numRetries++;
       }
   }
   
   if (numRetries == maxRetries) {
       throw std::runtime_error("Get Image fail! nRet [0x" + std::to_string(_nRet) + "]");
   }
   return _srcImage;
}

namespace {
    template<typename T>
    auto setCameraParam(void* handle, std::string_view paramName, T value)
        -> std::optional<int>
    {
        int nRet = MV_OK;
        if constexpr (std::is_same_v<T, float>) {
            nRet = MV_CC_SetFloatValue(handle, paramName.data(), value);
        } else if constexpr (std::is_same_v<T, int>) {
            nRet = MV_CC_SetIntValue(handle, paramName.data(), value);
        } else if constexpr (std::is_same_v<T, bool>) {
            nRet = MV_CC_SetBoolValue(handle, paramName.data(), value);
        } else if constexpr (std::is_same_v<T, std::string>) {
            nRet = MV_CC_SetEnumValueByString(handle, paramName.data(), value.c_str());
        }

        return nRet == MV_OK ? std::nullopt : std::make_optional(nRet);
    }

    template<typename T>
    auto getCameraParam(void* handle, std::string_view paramName)
        -> std::optional<T>
    {
        if constexpr (std::is_same_v<T, float>) {
            MVCC_FLOATVALUE value = {0};
            if (MV_CC_GetFloatValue(handle, paramName.data(), &value) == MV_OK) {
                return value.fCurValue;
            }
        } else if constexpr (std::is_same_v<T, int>) {
            MVCC_INTVALUE value = {0};
            if (MV_CC_GetIntValue(handle, paramName.data(), &value) == MV_OK) {
                return value.nCurValue;
            }
        } else if constexpr (std::is_same_v<T, bool>) {
            bool value = false;
            if (MV_CC_GetBoolValue(handle, paramName.data(), &value) == MV_OK) {
                return value;
            }
        }
        return std::nullopt;
    }
}

template<typename T>
bool HikCam::setParamValue(std::string_view name, T value) {
    if (auto error = setCameraParam(_handle, name, value)) {
        debug::print(
            debug::PrintMode::WARNING,
            "Camera",
            "Failed to set {}: [0x{:X}]",
            name,
            static_cast<uint32_t>(*error)
        );
        return false;
    }
    return true;
}

void HikCam::SetAttribute(CAM_INFO Info) {

    for (const auto& [name, param] : Info.parameters) {
        std::visit([this, &name](auto&& value) {
            if (name=="CamID") return;
            setParamValue(name, value);
        }, param);
    }
    fmt::print(
        fmt::fg(fmt::color::medium_purple),
        "===================Current Camera Setting====================\n"
    );
    check_and_print(Info);
    fmt::print(
        fmt::fg(fmt::color::medium_purple),
        "=============================================================\n"
    );


}


void HikCam::check_and_print(const CAM_INFO& Info) {
    const auto check_param = [](const auto& actual, const auto& expected, std::string_view name) {
        if constexpr (std::is_same_v<std::decay_t<decltype(actual)>, std::string>) {
            const bool isMatch = (actual == expected);
            if (isMatch) {
                fmt::print("{}: {}\n", name, actual);
            } else {
                fmt::print(fg(fmt::color::red), "   {}: {} (Expected: {})\n",
                           name, actual, expected);
            }
            return isMatch;
        } else {
            const bool isMatch = std::abs(static_cast<double>(actual) -
                                          static_cast<double>(expected)) < 0.1;
            if (isMatch) {
                fmt::print("{}: {}\n", name, actual);
            } else {
                fmt::print(fg(fmt::color::red), "{}: {} (Expected: {})\n",
                           name, actual, expected);
            }
            return isMatch;
        }
    };

    // 遍历所有参数进行验证
    for (const auto& [name, value] : Info.parameters) {
        std::visit([&](const auto& expected) {
            using T = std::decay_t<decltype(expected)>;
            if (auto actual = getCameraParam<T>(_handle, name)) {
                check_param(*actual, expected, name);
            }
        }, value);
    }
}

HikCam::~HikCam() {
   // ch:停止取流 | en:Stop grab image
   _nRet = MV_CC_StopGrabbing(_handle);
   if (MV_OK != _nRet) {
       std::cerr << "Stop Grabbing fail! nRet [0x" << std::hex << _nRet << std::dec << "]" << std::endl;
       //throw std::runtime_error("Stop Grabbing fail! nRet [0x" + std::to_string(_nRet) + "]");
   }

   // ch:注销抓图回调 | en:Unregister image callback
   _nRet = MV_CC_RegisterImageCallBackEx(_handle, NULL, NULL);
   if (MV_OK != _nRet) {
       std::cerr << "Unregister Image CallBack fail! nRet [0x" << std::hex << _nRet << std::dec << "]" << std::endl;
       //throw std::runtime_error("Unregister Image CallBack fail! nRet [0x" + std::to_string(_nRet) + "]");
   }

   // ch:关闭设备 | en:Close device
   _nRet = MV_CC_CloseDevice(_handle);
   if (MV_OK != _nRet) {
       std::cerr << "Close Device fail! nRet [0x" << std::hex << _nRet << std::dec << "]" << std::endl;
       //throw std::runtime_error("Close Device fail! nRet [0x" + std::to_string(_nRet) + "]");
   }

   // ch:销毁句柄 | en:Destroy handle
   _nRet = MV_CC_DestroyHandle(_handle);
   if (MV_OK != _nRet) {
       std::cerr << "Destroy _handle fail! nRet [0x" << std::hex << _nRet << std::dec << "]" << std::endl;
       //throw std::runtime_error("Destroy _handle fail! nRet [0x" + std::to_string(_nRet) + "]");
   }
   _handle = NULL;

   if (_handle != NULL) {
       MV_CC_DestroyHandle(_handle);
       _handle = NULL;
   }

   // ch:反初始化SDK | en:Finalize SDK
   //MV_CC_Finalize();
}

} // namespace camera