cmake_minimum_required(VERSION 3.16)

add_library(io STATIC
    camera.cpp
    hikrobot/hikrobot.cpp
    mindvision/mindvision.cpp
    usbcamera/usbcamera.cpp
    cboard/cboard.cpp
    imu/imu.cpp
    serial/serial.cpp
    gimbal/gimbal.cpp
)

target_include_directories(io PUBLIC
    ${PROJECT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/hikrobot/include
    ${CMAKE_CURRENT_SOURCE_DIR}/mindvision/include
    ${ARAVIS_INCLUDE_DIRS}
)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    target_link_directories(io PUBLIC hikrobot/lib/amd64 mindvision/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    target_link_directories(io PUBLIC hikrobot/lib/arm64 mindvision/lib/arm64)
else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

target_link_libraries(io PUBLIC
    tools
    Boost::boost
    Eigen3::Eigen
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
    opencv_videoio
    opencv_highgui
    opencv_calib3d
    opencv_dnn
    Threads::Threads
    MvCameraControl
    MVSDK
    usb-1.0
    ${ARAVIS_LIBRARIES}
)
