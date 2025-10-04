cmake_minimum_required(VERSION 3.16)
project(RMCV)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
find_program(CCACHE_PROGRAM ccache)
if (CCACHE_PROGRAM)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
endif ()
add_compile_options(-Wall -Werror=return-type -Wno-unused-variable)
if (CMAKE_BUILD_TYPE STREQUAL Release)
    add_compile_options(-O3 -flto -march=native)
elseif (CMAKE_BUILD_TYPE STREQUAL RelWithDebInfo)
    add_compile_options(-O3 -flto -march=native -g)
elseif (CMAKE_BUILD_TYPE STREQUAL MinSizeRel)
    add_compile_options(-Os -flto -march=native)
else ()
    add_compile_options(-O0 -g)
endif ()

# 设置 asset 和 log 目录路径
set(ASSET_DIR "${CMAKE_SOURCE_DIR}/asset")
set(CONFIG_DIR "${CMAKE_SOURCE_DIR}/config")
set(LOG_DIR "${CMAKE_SOURCE_DIR}/log")


# 添加全局编译定义 - 所有目标都会自动继承
add_compile_definitions(
    ASSET_DIR="${ASSET_DIR}"
    CONFIG_DIR="${CONFIG_DIR}"
    LOG_DIR="${LOG_DIR}"
)

message(STATUS "--------------------CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}--------------------")

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tomlplusplus CONFIG REQUIRED)
find_package(Python 3.6 REQUIRED COMPONENTS Development Interpreter)
find_package(pybind11 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${Python_INCLUDE_DIRS})
include_directories(${pybind11_INCLUDE_DIRS})
include_directories(umt)

add_subdirectory(plugin)
add_subdirectory(hardware)

add_executable(RMCV2026 main.cpp)
target_link_libraries(RMCV2026 ${OpenCV_LIBS} fmt::fmt plugin hardware)

add_executable(test_param test/test_param.cpp)
target_link_libraries(test_param ${OpenCV_LIBS} fmt::fmt plugin)


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
if(Python_FOUND)
    message(STATUS "Python Include Dirs: ${Python_INCLUDE_DIRS}")
    message(STATUS "Python Libraries: ${Python_LIBRARIES}")
endif()

# --- pybind11 ---
if(pybind11_FOUND)
    # pybind11 提供了一个便捷变量来获取头文件路径
    message(STATUS "pybind11 Include Dirs: ${pybind11_INCLUDE_DIRS}")
endif()

message(STATUS "-------------------------------------------------------------------")
