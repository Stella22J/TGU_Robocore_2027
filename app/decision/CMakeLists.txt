add_library(decision OBJECT
        decider.cpp
        shooter.cpp
        voter.cpp
        multithread/commandgener.cpp
        multithread/mt_detector.cpp
)

target_include_directories(decision PUBLIC
        ${PROJECT_SOURCE_DIR}
        ${APP_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(decision PUBLIC
        auto_aim
        tracker
        predictor
        io
        tools
        ${OpenCV_LIBS}
        Eigen3::Eigen
        openvino::runtime
        yaml-cpp::yaml-cpp
        spdlog::spdlog
        fmt::fmt
)