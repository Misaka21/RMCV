set(hikrobot "/opt/MVS")
aux_source_directory(./hik_cam hik_camera_src)

add_library(hardware STATIC ${hik_camera_src})

# hikrobot
target_include_directories(hardware PUBLIC ${hikrobot}/include/)
if (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    target_link_directories(hardware PUBLIC ${hikrobot}/lib/64/)
elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    target_link_directories(hardware PUBLIC ${hikrobot}/lib/aarch64/)
else ()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif ()

# Link MVS library
if (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    target_link_libraries(hardware PUBLIC MvCameraControl)
elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    target_link_libraries(hardware PUBLIC MvCameraControl)
endif ()
