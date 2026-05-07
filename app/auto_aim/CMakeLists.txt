add_library(auto_aim OBJECT
        armor.cpp
        classifier.cpp
        detector.cpp
        perceptron.cpp
        solver.cpp
        yolo.cpp
        yolos/yolov5.cpp
        yolos/yolov8.cpp
        yolos/yolo11.cpp
)

target_include_directories(auto_aim PUBLIC
        ${PROJECT_SOURCE_DIR}
        ${APP_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(auto_aim PUBLIC
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