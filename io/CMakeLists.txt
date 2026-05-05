cmake_minimum_required(VERSION 3.16)

add_library(io STATIC
        camera.cpp
        hikrobot/hikrobot.cpp
        mindvision/mindvision.cpp
        usbcamera/usbcamera.cpp
        cboard/cboard.cpp
        imu/imu.cpp
        serial/serial.cpp
)

target_include_directories(io
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/hikrobot/include
        ${CMAKE_CURRENT_SOURCE_DIR}/mindvision/include
)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(io PUBLIC hikrobot/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(io PUBLIC hikrobot/lib/arm64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(io PUBLIC mindvision/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(io PUBLIC mindvision/lib/arm64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED CONFIG)

target_link_directories(io
        PUBLIC
        ${Boost_INCLUDE_DIRS}
)

target_link_libraries(io
        PUBLIC
        Eigen3::Eigen
        ${OpenCV_LIBS}
        Threads::Threads
        HikrobotSDK
        MindvisionSDK
        usb-1.0
)