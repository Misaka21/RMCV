set(hikrobot "/opt/MVS")

# ------------------------------------------------------------------------------
# HikRobot MVS SDK backend (optional)
# - Linux x86_64/aarch64: use /opt/MVS if installed
# - Other platforms (macOS/Windows/arm64, etc.): build an OpenCV VideoCapture fallback
# ------------------------------------------------------------------------------

set(HIK_SDK_HEADER "${hikrobot}/include/MvCameraControl.h")
set(RMCV_HIK_BACKEND_AVAILABLE OFF)
if(EXISTS "${HIK_SDK_HEADER}")
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
        set(RMCV_HIK_BACKEND_AVAILABLE ON)
    endif()
endif()

option(RMCV_WITH_HIK_CAMERA "Build with HikRobot MVS camera backend (/opt/MVS)" ${RMCV_HIK_BACKEND_AVAILABLE})
if(RMCV_WITH_HIK_CAMERA AND NOT RMCV_HIK_BACKEND_AVAILABLE)
    message(FATAL_ERROR "RMCV_WITH_HIK_CAMERA=ON but HikRobot MVS SDK is unavailable for ${CMAKE_SYSTEM_PROCESSOR} (expected ${HIK_SDK_HEADER}).")
endif()

if(RMCV_WITH_HIK_CAMERA)
    add_library(hardware_camera STATIC
        hik_camera.cpp
    )
    target_compile_definitions(hardware_camera PUBLIC RMCV_WITH_HIK_CAMERA=1)

    target_include_directories(hardware_camera PUBLIC "${hikrobot}/include/")

    # 根据架构设置库路径
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
        target_link_directories(hardware_camera PUBLIC "${hikrobot}/lib/64/")
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
        target_link_directories(hardware_camera PUBLIC "${hikrobot}/lib/aarch64/")
    endif()

    target_link_libraries(hardware_camera PUBLIC
        MvCameraControl
        ${OpenCV_LIBS}
        fmt::fmt
    )
else()
    message(STATUS "HikRobot MVS SDK backend disabled/unavailable - using OpenCV VideoCapture fallback camera.")
    add_library(hardware_camera STATIC
        hik_camera_fallback.cpp
    )
    target_compile_definitions(hardware_camera PUBLIC RMCV_WITH_HIK_CAMERA=0)
    target_link_libraries(hardware_camera PUBLIC
        ${OpenCV_LIBS}
        fmt::fmt
    )
endif()
