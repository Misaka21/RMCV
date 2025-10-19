aux_source_directory(. serial_src)

# 创建串口静态库
add_library(hardware_serial STATIC ${serial_src})

# 设置包含目录
target_include_directories(hardware_serial PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# 查找并链接libusb-1.0依赖
find_path(LIBUSB_INCLUDE_DIR
    NAMES libusb.h
    PATHS
        /usr/include/libusb-1.0
        /usr/local/include/libusb-1.0
        /opt/homebrew/include/libusb-1.0
    PATH_SUFFIXES libusb-1.0
)

find_library(LIBUSB_LIBRARY
    NAMES usb-1.0 libusb-1.0
    PATHS
        /usr/lib
        /usr/local/lib
        /opt/homebrew/lib
)

if(LIBUSB_INCLUDE_DIR AND LIBUSB_LIBRARY)
    target_include_directories(hardware_serial PRIVATE ${LIBUSB_INCLUDE_DIR})
    target_link_libraries(hardware_serial PRIVATE ${LIBUSB_LIBRARY})
    target_compile_definitions(hardware_serial PRIVATE HAVE_LIBUSB_1_0)
else()
    message(WARNING "libusb-1.0 not found, USB bulk transfer support will be disabled")
endif()

# 链接系统库
target_link_libraries(hardware_serial PRIVATE
    pthread  # 用于多线程支持
)

# 平台特定设置
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_libraries(hardware_serial PRIVATE
        rt  # 用于时钟函数
    )
endif()