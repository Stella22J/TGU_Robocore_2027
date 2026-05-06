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
        opencv_core
        opencv_imgproc
        opencv_imgcodecs
        opencv_videoio
        opencv_highgui
        opencv_calib3d
        opencv_dnn
        Eigen3::Eigen
        openvino::runtime
        spdlog::spdlog
        fmt::fmt
)