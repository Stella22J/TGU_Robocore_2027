set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")
find_package(OpenVINO REQUIRED COMPONENTS Runtime)

set(APP_INCLUDE_DIRS
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/auto_aim
        ${CMAKE_CURRENT_SOURCE_DIR}/auto_aim/yolos
        ${CMAKE_CURRENT_SOURCE_DIR}/tracker
        ${CMAKE_CURRENT_SOURCE_DIR}/predictor
        ${CMAKE_CURRENT_SOURCE_DIR}/predictor/planner
        ${CMAKE_CURRENT_SOURCE_DIR}/decision
        ${CMAKE_CURRENT_SOURCE_DIR}/decision/multithread
)

add_subdirectory(auto_aim)
add_subdirectory(tracker)
add_subdirectory(predictor)
add_subdirectory(decision)
# add_subdirectory(navigation)