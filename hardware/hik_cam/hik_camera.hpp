#ifndef HIK_CAMERA_H
#define HIK_CAMERA_H

#include "MvCameraControl.h"
#include "plugin/debug/logger.hpp"
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <cstdio>
#include <fmt/core.h>
#include <iostream>
#include <unordered_map>
#include <variant>

namespace camera {


void __stdcall ImageCallBackEx(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser);

class HikCam {
public:
    HikCam();

    void open();

    ~HikCam();

    auto capture() -> cv::Mat&;
    int frame_id;

private:
    uint32_t _nRet = MV_OK;
    void* _handle = NULL;
    unsigned char* _pDstData = NULL;
    cv::Mat _srcImage;
    template<typename T>
    bool setParamValue(std::string_view name, T value);
    bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo);
    void check_and_print(const CAM_INFO& Info);
    void SetAttribute(CAM_INFO Info);
};

} // namespace camera
#endif // HIK_CAMERA_H