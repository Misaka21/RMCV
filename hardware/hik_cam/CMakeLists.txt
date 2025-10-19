set(hikrobot "/opt/MVS")
aux_source_directory(. hik_camera_src)

# 创建相机静态库
add_library(hardware_camera STATIC ${hik_camera_src})

# HIKRobot 相机依赖
target_include_directories(hardware_camera PUBLIC ${hikrobot}/include/)
if (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    target_link_directories(hardware_camera PUBLIC ${hikrobot}/lib/64/)
elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    target_link_directories(hardware_camera PUBLIC ${hikrobot}/lib/aarch64/)
else ()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif ()

# 链接 MVS 库
if (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    target_link_libraries(hardware_camera PUBLIC MvCameraControl)
elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    target_link_libraries(hardware_camera PUBLIC MvCameraControl)
endif ()

# 添加 OpenCV 依赖 (来自父目录)
target_link_libraries(hardware_camera PUBLIC ${OpenCV_LIBS})