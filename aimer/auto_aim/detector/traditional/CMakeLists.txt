# traditional detector library
add_library(detector_traditional STATIC
    armor_detector.cpp
    number_classifier.cpp
    light_corner_corrector.cpp
)

target_include_directories(detector_traditional PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

find_package(TBB REQUIRED)

target_link_libraries(detector_traditional PUBLIC
    ${OpenCV_LIBS}
    Eigen3::Eigen
    fmt::fmt
    TBB::tbb
)
