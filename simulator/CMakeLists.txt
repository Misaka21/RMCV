# Simulator模块 - ROS2模拟器接入
# 需要ROS2环境: source /opt/ros/humble/setup.bash

find_package(rclcpp QUIET)
find_package(sensor_msgs QUIET)
find_package(geometry_msgs QUIET)
find_package(cv_bridge QUIET)

if(rclcpp_FOUND AND sensor_msgs_FOUND AND geometry_msgs_FOUND AND cv_bridge_FOUND)
    message(STATUS "ROS2 found, building simulator module")

    add_library(simulator STATIC
        simulator_node.cpp
    )

    target_include_directories(simulator PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}
    )

    target_link_libraries(simulator
        ${OpenCV_LIBS}
        fmt::fmt
        Eigen3::Eigen
        plugin
        hardware
    )

    ament_target_dependencies(simulator
        rclcpp
        sensor_msgs
        geometry_msgs
        cv_bridge
    )

    # 导出变量供主CMakeLists使用
    set(SIMULATOR_FOUND TRUE PARENT_SCOPE)

else()
    message(WARNING "ROS2 packages not found, simulator module will not be built")
    message(WARNING "  To enable: source /opt/ros/humble/setup.bash before cmake")
    set(SIMULATOR_FOUND FALSE PARENT_SCOPE)
endif()
