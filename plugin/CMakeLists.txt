cmake_minimum_required(VERSION 3.16)
set(lib_name "plugin")
# Find dependencies
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tomlplusplus CONFIG REQUIRED)


# Collect debug source
aux_source_directory(./debug debug_src)
aux_source_directory(./param param_src)

# Create plugin library
add_library(${lib_name} STATIC ${debug_src} ${param_src})

target_include_directories(${lib_name} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(${lib_name} PUBLIC
        ${OpenCV_LIBS}
        Eigen3::Eigen
        tomlplusplus::tomlplusplus
)
