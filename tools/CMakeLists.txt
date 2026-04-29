cmake_minimum_required(VERSION 3.16)

add_library(tools STATIC
    exiter.cpp
    extended_kalman_filter.cpp
    ransac_sine_fitter.cpp
    img_tools.cpp
    math_tools.cpp
    plotter.cpp
    trajectory.cpp
    recorder.cpp
    logger.cpp
    pid.cpp
    crc.cpp
)

target_include_directories(tools PUBLIC
    ${PROJECT_SOURCE_DIR}
)

target_link_libraries(tools
    PUBLIC
    Eigen3::Eigen
    ${OpenCV_LIBS}
    spdlog::spdlog
    nlohmann_json::nlohmann_json
    Threads::Threads
    fmt::fmt
)

