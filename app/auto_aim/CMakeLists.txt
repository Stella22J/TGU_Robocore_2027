add_library(auto_aim OBJECT
        armor.cpp
        auto_aim.cpp
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
        ${OpenCV_LIBS}
        Eigen3::Eigen
        openvino::runtime
        yaml-cpp::yaml-cpp
        spdlog::spdlog
        fmt::fmt
)