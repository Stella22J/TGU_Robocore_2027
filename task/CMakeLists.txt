# sentry
add_executable(sentry sentry.cpp)
add_executable(sentry_bp sentry_bp.cpp)
add_executable(sentry_debug sentry_debug.cpp)
add_executable(sentry_multithread sentry_multithread.cpp)

target_link_libraries(sentry PRIVATE io app tools ${OpenCV_LIBS})
target_link_libraries(sentry_bp PRIVATE io app tools ${OpenCV_LIBS})
target_link_libraries(sentry_debug PRIVATE io app tools ${OpenCV_LIBS})
target_link_libraries(sentry_multithread PRIVATE io app tools ${OpenCV_LIBS})


# tests
add_executable(read_toml test/read_toml.cpp)
add_executable(test_logger test/test_logger.cpp)
add_executable(test_foxglove test/test_foxglove.cpp)
add_executable(test_camera test/test_camera.cpp)

set(SERIAL_SRC "${CMAKE_SOURCE_DIR}/io/serial/serial.cpp")

add_executable(test_serial
        test/test_serial.cpp
        "${SERIAL_SRC}"
)

add_dependencies(test_serial tools)

target_link_libraries(test_serial
        PRIVATE
        $<TARGET_FILE:tools>
        spdlog::spdlog
        fmt::fmt
)

add_dependencies(test_logger tools)

target_link_libraries(test_logger
        PRIVATE
        $<TARGET_FILE:tools>
        spdlog::spdlog
        fmt::fmt
)

target_link_libraries(test_camera
        PRIVATE
        ${ARAVIS_LIBRARIES}
        opencv_core
        opencv_imgproc
        opencv_imgcodecs
)