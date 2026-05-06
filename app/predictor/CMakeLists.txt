add_subdirectory(planner/tinympc)

add_library(predictor OBJECT
        aimer.cpp
        planner/planner.cpp
)

target_include_directories(predictor PUBLIC
        ${PROJECT_SOURCE_DIR}
        ${APP_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/planner/tinympc
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(predictor PUBLIC
        auto_aim
        tracker
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
        tinympcstatic
        spdlog::spdlog
        fmt::fmt
)