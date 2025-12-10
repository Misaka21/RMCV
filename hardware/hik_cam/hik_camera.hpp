#ifndef HIK_CAMERA_H
#define HIK_CAMERA_H

// C system headers
#include <cstdio>

// C++ system headers
#include <iostream>
#include <string>
#include <unordered_map>
#include <variant>

// Third-party library headers
#include <MvCameraControl.h>
#include <fmt/core.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

// Project headers
#include "hik_log.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/param/static_config.hpp"

namespace camera {
    using CAM_INFO = std::variant<bool, int64_t, double, std::string>;

    // 常量定义
    constexpr int MAX_RETRY_ATTEMPTS = 3;
    constexpr int RETRY_DELAY_SECONDS = 5;
    constexpr int INFO_BUFFER_SIZE = INFO_MAX_BUFFER_SIZE;

    class HikCam {
    public:
        HikCam();

        void open();

        ~HikCam();

        auto capture() -> cv::Mat &;

        int frame_id;

    private:
        uint32_t _nRet = MV_OK;
        void *_handle = NULL;
        unsigned char *_pDstData = NULL;
        cv::Mat _srcImage;
        std::vector<std::pair<std::string, CAM_INFO> > _param_from_toml;

        bool _use_camera_sn;
        std::string _camera_sn;
        bool _use_mfs_config;
        std::string _mfs_config_path;
        bool _use_toml_config;

        bool _print_device_info(MV_CC_DEVICE_INFO *pstMVDevInfo);

        void _check_and_print();

        void _set_camera_info_batch();

        template<typename T>
        auto _get_camera_param(std::string_view param_name) -> std::optional<T>;

        // 重构后的私有方法
        void _enumerate_devices(MV_CC_DEVICE_INFO_LIST &deviceList);

        bool _find_device_by_sn(const std::string &sn, const MV_CC_DEVICE_INFO_LIST &deviceList, int &deviceIndex);

        bool _open_camera_by_sn(const std::string &sn, MV_CC_DEVICE_INFO_LIST &deviceList, int &deviceIndex);

        bool _open_camera_by_index(int deviceIndex, const MV_CC_DEVICE_INFO_LIST &deviceList);

        void _configure_gige_device(const MV_CC_DEVICE_INFO *deviceInfo);

        void _load_camera_config();


        //setvalue重载
        //enum
        inline void set_camera_info(std::string key, std::string value) {
            HIKCAM_WARN(MV_CC_SetEnumValueByString(this->_handle, key.c_str(), value.c_str()));
        }

        inline void set_camera_info(std::string key, int64_t value) {
            HIKCAM_WARN(MV_CC_SetIntValue(this->_handle, key.c_str(), static_cast<int>(value)));
        }

        inline void set_camera_info(std::string key, double value) {
            HIKCAM_WARN(MV_CC_SetFloatValue(this->_handle, key.c_str(), static_cast<float>(value)));
        }

        inline void set_camera_info(std::string key, bool value) {
            HIKCAM_WARN(MV_CC_SetBoolValue(this->_handle, key.c_str(), value));
        }
    };
} // namespace camera
#endif // HIK_CAMERA_H
