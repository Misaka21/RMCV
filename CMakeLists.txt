cmake_minimum_required(VERSION 3.16)
project(RMCV)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_BUILD_TYPE Release)

# VTune 性能分析选项
option(ENABLE_PROFILING "Enable profiling support for VTune/perf" OFF)

find_program(CCACHE_PROGRAM ccache)
if (CCACHE_PROGRAM)
    message(STATUS "ccache found - compilation will be cached for faster rebuilds")
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
else ()
    message(STATUS "ccache not found - to speed up compilation, install it:")
    message(STATUS "  Ubuntu/Debian: sudo apt-get install ccache")
    message(STATUS "  macOS: brew install ccache")
endif ()
# C/C++ 编译选项 (不应用于 CUDA)
add_compile_options(
    $<$<COMPILE_LANGUAGE:C,CXX>:-Wall>
    $<$<COMPILE_LANGUAGE:C,CXX>:-Werror=return-type>
    $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-unused-variable>
)

# 性能分析模式：保持优化但添加调试符号和帧指针
if (ENABLE_PROFILING)
    message(STATUS "Profiling mode enabled - adding debug symbols and frame pointers")
    add_compile_options(
        $<$<COMPILE_LANGUAGE:C,CXX>:-O3>
        $<$<COMPILE_LANGUAGE:C,CXX>:-g>
        $<$<COMPILE_LANGUAGE:C,CXX>:-fno-omit-frame-pointer>
        $<$<COMPILE_LANGUAGE:C,CXX>:-march=native>
    )
    # 不使用 LTO，便于符号解析
elseif (CMAKE_BUILD_TYPE STREQUAL Release)
    # LTO 只对 C/CXX 启用，CUDA 不支持
    add_compile_options(
        $<$<COMPILE_LANGUAGE:C,CXX>:-O3>
        $<$<COMPILE_LANGUAGE:C,CXX>:-march=native>
    )
    # 注意: 移除 LTO 以避免与 CUDA 冲突
    # add_compile_options($<$<COMPILE_LANGUAGE:C,CXX>:-flto=auto>)
elseif (CMAKE_BUILD_TYPE STREQUAL RelWithDebInfo)
    add_compile_options(
        $<$<COMPILE_LANGUAGE:C,CXX>:-O3>
        $<$<COMPILE_LANGUAGE:C,CXX>:-march=native>
        $<$<COMPILE_LANGUAGE:C,CXX>:-g>
    )
elseif (CMAKE_BUILD_TYPE STREQUAL MinSizeRel)
    add_compile_options(
        $<$<COMPILE_LANGUAGE:C,CXX>:-Os>
        $<$<COMPILE_LANGUAGE:C,CXX>:-march=native>
    )
else ()
    add_compile_options($<$<COMPILE_LANGUAGE:C,CXX>:-O0> $<$<COMPILE_LANGUAGE:C,CXX>:-g>)
endif ()

# 设置 asset 和 log 目录路径
set(ASSET_DIR "${CMAKE_SOURCE_DIR}/asset")
set(CONFIG_DIR "${CMAKE_SOURCE_DIR}/config")
set(LOG_DIR "${CMAKE_SOURCE_DIR}/log")
set(WEB_DIR "${CMAKE_SOURCE_DIR}/web")


# 添加全局编译定义 - 所有目标都会自动继承
add_compile_definitions(
    ASSET_DIR="${ASSET_DIR}"
    CONFIG_DIR="${CONFIG_DIR}"
    LOG_DIR="${LOG_DIR}"
    WEB_DIR="${WEB_DIR}"
)

message(STATUS "--------------------CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}--------------------")
message(STATUS "--------------------ENABLE_PROFILING: ${ENABLE_PROFILING}--------------------")

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(tomlplusplus CONFIG REQUIRED)

# Python/pybind11 可选 - 用于 Web UI (--web 参数)
option(ENABLE_WEBVIEW "Enable Web UI (requires Python and pybind11)" ON)
if(ENABLE_WEBVIEW)
    find_package(Python 3.6 QUIET COMPONENTS Development Interpreter)
    find_package(pybind11 QUIET)
    if(pybind11_FOUND AND Python_FOUND)
        add_compile_definitions(ENABLE_WEBVIEW)
        message(STATUS "Web UI: ENABLED (Python ${Python_VERSION}, pybind11 found)")
    else()
        set(ENABLE_WEBVIEW OFF)
        message(STATUS "Web UI: DISABLED (pybind11 or Python not found)")
    endif()
else()
    message(STATUS "Web UI: DISABLED (by option)")
endif()

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})
if(ENABLE_WEBVIEW)
    include_directories(${Python_INCLUDE_DIRS})
    include_directories(${pybind11_INCLUDE_DIRS})
endif()
include_directories(umt)

add_subdirectory(plugin)
add_subdirectory(hardware)
add_subdirectory(aimer)
add_subdirectory(simulator)

add_executable(RMCV2026 main.cpp)
# 使用 whole-archive 确保 pybind11 嵌入模块不被链接器优化掉
if(ENABLE_WEBVIEW)
    target_link_libraries(RMCV2026
        ${OpenCV_LIBS} fmt::fmt
        -Wl,--whole-archive plugin -Wl,--no-whole-archive
        hardware detector predictor autoaim_fire_control buff_detector rmcv_bag
        ${Python_LIBRARIES}
    )
else()
    target_link_libraries(RMCV2026 ${OpenCV_LIBS} fmt::fmt plugin hardware detector predictor autoaim_fire_control buff_detector rmcv_bag)
endif()

add_executable(test_param test/test_param.cpp)
target_link_libraries(test_param ${OpenCV_LIBS} fmt::fmt plugin Threads::Threads)

add_executable(test_serial test/test_serial.cpp)
target_link_libraries(test_serial ${OpenCV_LIBS} fmt::fmt plugin hardware)

add_executable(test_camera test/test_camera.cpp)
target_link_libraries(test_camera ${OpenCV_LIBS} fmt::fmt plugin hardware)

add_executable(test_hardware test/test_hardware.cpp)
target_link_libraries(test_hardware ${OpenCV_LIBS} fmt::fmt plugin hardware)

add_executable(test_transformer test/test_transformer.cpp)
target_link_libraries(test_transformer ${OpenCV_LIBS} fmt::fmt aimer_common)

add_executable(test_imu test/test_imu.cpp)
target_link_libraries(test_imu ${OpenCV_LIBS} fmt::fmt plugin hardware)

add_executable(test_ground_plane test/test_ground_plane.cpp)
target_link_libraries(test_ground_plane ${OpenCV_LIBS} fmt::fmt plugin hardware aimer_common)

add_executable(1.test_gimbal2imubody test/test_gimbal2imubody.cpp)
target_link_libraries(1.test_gimbal2imubody ${OpenCV_LIBS} fmt::fmt hardware aimer_common)

add_executable(test_ballistic test/test_ballistic.cpp)
target_link_libraries(test_ballistic ${OpenCV_LIBS} fmt::fmt plugin hardware aimer_common)

add_executable(test_calibration test/test_calibration.cpp)
target_link_libraries(test_calibration ${OpenCV_LIBS} fmt::fmt plugin hardware)

add_executable(test_playback test/test_playback.cpp)
target_link_libraries(test_playback ${OpenCV_LIBS} fmt::fmt plugin hardware detector predictor rmcv_bag)

# 相机-IMU时间戳标定 (需要Ceres)
find_package(Ceres QUIET)
if(Ceres_FOUND)
    add_executable(3.test_time_sync test/time_sync/test_time_sync.cpp)
    target_include_directories(3.test_time_sync PRIVATE ${PROJECT_SOURCE_DIR}/test ${CERES_INCLUDE_DIRS})
    target_link_libraries(3.test_time_sync ${OpenCV_LIBS} fmt::fmt plugin hardware detector aimer_common ${CERES_LIBRARIES})
    message(STATUS "3.test_time_sync will be built (Ceres found)")

    # 相机外参自动标定 (使用Ceres)
    add_executable(2.test_extrinsic_calib test/extrinsic_calib/test_extrinsic_calib.cpp)
    target_include_directories(2.test_extrinsic_calib PRIVATE ${PROJECT_SOURCE_DIR}/test ${CERES_INCLUDE_DIRS})
    target_link_libraries(2.test_extrinsic_calib ${OpenCV_LIBS} fmt::fmt plugin hardware aimer_common ${CERES_LIBRARIES})
    message(STATUS "2.test_extrinsic_calib will be built (Ceres found)")
else()
    message(STATUS "3.test_time_sync will NOT be built (Ceres not found)")

    # 相机外参自动标定 (无Ceres，只用网格搜索)
    add_executable(2.test_extrinsic_calib test/extrinsic_calib/test_extrinsic_calib.cpp)
    target_include_directories(2.test_extrinsic_calib PRIVATE ${PROJECT_SOURCE_DIR}/test)
    target_link_libraries(2.test_extrinsic_calib ${OpenCV_LIBS} fmt::fmt plugin hardware aimer_common)
    message(STATUS "2.test_extrinsic_calib will be built (without Ceres, grid search only)")
endif()

# 海康时间戳验证工具 (不需要Ceres)
add_executable(test_timestamp test/time_sync/test_timestamp.cpp)
target_include_directories(test_timestamp PRIVATE ${PROJECT_SOURCE_DIR}/test)
target_link_libraries(test_timestamp ${OpenCV_LIBS} fmt::fmt plugin hardware)

# 相机-IMU时间戳顺序检查工具 (不需要Ceres)
add_executable(check_timestamp_order test/time_sync/check_timestamp_order.cpp)
target_include_directories(check_timestamp_order PRIVATE ${PROJECT_SOURCE_DIR}/test)
target_link_libraries(check_timestamp_order ${OpenCV_LIBS} fmt::fmt plugin hardware)

# 火控可视化测试 (默认使用实车硬件，可选模拟器)
add_executable(test_fire_control test/test_fire_control.cpp)
target_link_libraries(test_fire_control ${OpenCV_LIBS} fmt::fmt plugin hardware detector predictor autoaim_fire_control)
if(SIMULATOR_FOUND)
    target_compile_definitions(test_fire_control PRIVATE SIMULATOR_FOUND)
    target_link_libraries(test_fire_control simulator)
endif()

# TensorRT Engine 生成测试 (需要TensorRT)
if(ENABLE_TENSORRT_DETECTOR)
    add_executable(test_trt_engine test/test_trt_engine.cpp)
    target_link_libraries(test_trt_engine ${OpenCV_LIBS} fmt::fmt plugin detector_tensorrt)
    message(STATUS "test_trt_engine will be built (TensorRT enabled)")
endif()

# Simulator测试 (需要ROS2环境)
if(SIMULATOR_FOUND)
    add_executable(test_simulator test/test_simulator.cpp)
    target_link_libraries(test_simulator ${OpenCV_LIBS} fmt::fmt plugin hardware simulator detector predictor)
    message(STATUS "test_simulator will be built (ROS2 found)")
endif()


# ... (在你现有的 add_subdirectory 之后)

# ==============================================================================
# 调试信息：打印依赖库的详细路径
# ==============================================================================
message(STATUS "-------------------- Dependency Debug Information --------------------")

# --- OpenCV ---
if(OpenCV_FOUND)
    message(STATUS "OpenCV Version: ${OpenCV_VERSION}")
    message(STATUS "OpenCV Libraries: ${OpenCV_LIBS}")
    message(STATUS "OpenCV Include Dirs: ${OpenCV_INCLUDE_DIRS}")
endif()

# --- fmt ---
if(TARGET fmt::fmt)
    get_target_property(FMT_INCLUDE_DIR fmt::fmt INTERFACE_INCLUDE_DIRECTORIES)
    get_target_property(FMT_LIBRARY_LOCATION fmt::fmt IMPORTED_LOCATION)
    message(STATUS "fmt Library Location: ${FMT_LIBRARY_LOCATION}")
    message(STATUS "fmt Include Dirs: ${FMT_INCLUDE_DIR}")
endif()

# --- Eigen3 ---
if(Eigen3_FOUND)
    message(STATUS "Eigen3 Include Dir: ${EIGEN3_INCLUDE_DIR}")
endif()

# --- tomlplusplus ---
if(TARGET tomlplusplus::tomlplusplus)
    get_target_property(TOMLPP_INCLUDE_DIR tomlplusplus::tomlplusplus INTERFACE_INCLUDE_DIRECTORIES)
    get_target_property(TOMLPP_LIBRARY_LOCATION tomlplusplus::tomlplusplus IMPORTED_LOCATION)
    message(STATUS "tomlplusplus Library Location: ${TOMLPP_LIBRARY_LOCATION}")
    message(STATUS "tomlplusplus Include Dirs: ${TOMLPP_INCLUDE_DIR}")
endif()

# --- Python ---
if(ENABLE_WEBVIEW)
    message(STATUS "Python Include Dirs: ${Python_INCLUDE_DIRS}")
    message(STATUS "Python Libraries: ${Python_LIBRARIES}")
    message(STATUS "pybind11 Include Dirs: ${pybind11_INCLUDE_DIRS}")
endif()

message(STATUS "-------------------------------------------------------------------")
