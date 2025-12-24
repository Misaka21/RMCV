#ifndef HIK_CAMERA_H
#define HIK_CAMERA_H

// C system headers
#include <cstdio>

// C++ system headers
#include <iostream>
#include <optional>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

// Third-party library headers
#include <MvCameraControl.h>
#include <fmt/core.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

// Project headers
#include "hik_log.hpp"
#include "plugin/debug/logger.hpp"

namespace camera {

// ============================================================================
// Camera Configuration Structure
// ============================================================================

/**
 * @brief Camera parameter type for runtime configuration
 * Supports bool, int64_t, double, string types
 */
using CameraParam = std::variant<bool, int64_t, double, std::string>;

/**
 * @brief Camera configuration structure
 * Decouples HikCam from TOML configuration
 */
struct CameraConfig {
    // Device selection
    bool use_camera_sn = false;
    std::string camera_sn;

    // MFS config file
    bool use_mfs_config = false;
    std::string mfs_config_path;  // Full path to .mfs file

    // Runtime parameters (from TOML Camera.config section)
    bool use_runtime_config = false;
    std::vector<std::pair<std::string, CameraParam>> runtime_params;
};

// ============================================================================
// Constants
// ============================================================================

constexpr int MAX_RETRY_ATTEMPTS = 3;
constexpr int RETRY_DELAY_SECONDS = 3;
constexpr int INFO_BUFFER_SIZE = INFO_MAX_BUFFER_SIZE;

// ============================================================================
// HikCam Class
// ============================================================================

class HikCam {
public:
    /**
     * @brief Construct HikCam with configuration
     * @param config Camera configuration struct
     */
    explicit HikCam(const CameraConfig& config);

    /**
     * @brief Default constructor (uses default config)
     */
    HikCam();

    ~HikCam();

    /**
     * @brief Open camera and start grabbing
     */
    void open();

    /**
     * @brief Capture one frame
     * @return Reference to captured image
     */
    auto capture() -> cv::Mat&;

    int frame_id = 0;
    int64_t host_timestamp = 0;  // 主机时间戳 (nHostTimeStamp)

private:
    CameraConfig _config;
    uint32_t _nRet = MV_OK;
    void* _handle = nullptr;
    unsigned char* _pDstData = nullptr;
    cv::Mat _srcImage;

    bool _print_device_info(MV_CC_DEVICE_INFO* pstMVDevInfo);
    void _check_and_print();
    void _set_camera_info_batch();

    template <typename T>
    auto _get_camera_param(std::string_view param_name) -> std::optional<T>;

    bool _enumerate_devices(MV_CC_DEVICE_INFO_LIST& deviceList);
    bool _find_device_by_sn(const std::string& sn, const MV_CC_DEVICE_INFO_LIST& deviceList, int& deviceIndex);
    bool _open_camera_by_sn(const std::string& sn, MV_CC_DEVICE_INFO_LIST& deviceList, int& deviceIndex);
    bool _open_camera_by_index(int deviceIndex, const MV_CC_DEVICE_INFO_LIST& deviceList);
    void _configure_gige_device(const MV_CC_DEVICE_INFO* deviceInfo);
    void _load_camera_config();

    // Set value overloads
    inline void set_camera_info(const std::string& key, const std::string& value) {
        HIKCAM_WARN(MV_CC_SetEnumValueByString(this->_handle, key.c_str(), value.c_str()));
    }

    inline void set_camera_info(const std::string& key, int64_t value) {
        HIKCAM_WARN(MV_CC_SetIntValue(this->_handle, key.c_str(), static_cast<int>(value)));
    }

    inline void set_camera_info(const std::string& key, double value) {
        HIKCAM_WARN(MV_CC_SetFloatValue(this->_handle, key.c_str(), static_cast<float>(value)));
    }

    inline void set_camera_info(const std::string& key, bool value) {
        HIKCAM_WARN(MV_CC_SetBoolValue(this->_handle, key.c_str(), value));
    }
};

}  // namespace camera

#endif  // HIK_CAMERA_H
