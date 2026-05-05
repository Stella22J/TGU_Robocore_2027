add_library(tracker OBJECT
        target.cpp
        tracker.cpp
)

target_include_directories(tracker PUBLIC
        ${PROJECT_SOURCE_DIR}
        ${APP_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(tracker PUBLIC
        auto_aim
        io
        tools
        ${OpenCV_LIBS}
        Eigen3::Eigen
        spdlog::spdlog
        fmt::fmt
)