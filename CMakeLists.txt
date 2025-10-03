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
set(LOG_DIR "${CMAKE_SOURCE_DIR}/log")


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


add_executable(RMCV2026 main.cpp)

target_link_libraries(RMCV2026 ${OpenCV_LIBS} fmt::fmt plugin)


target_compile_definitions(RMCV2026 PRIVATE
        ASSET_DIR="${ASSET_DIR}"
        LOG_DIR="${LOG_DIR}"
)