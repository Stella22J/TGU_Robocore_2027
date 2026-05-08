/**
 * @file camera.cpp
 * @brief 实现相机统一入口和TOML配置解析。
 */

#include "camera.hpp"

#include <stdexcept>
#include <string>

#include "hikrobot/hikrobot.hpp"
#include "mindvision/mindvision.hpp"
#include "tools/toml.hpp"

namespace io {

Camera::Camera(const std::string& config_path) {
    const auto config = tools::load(config_path);
    const auto camera_name = tools::read<std::string>(config, "camera_name");
    const auto exposure_ms = tools::read<double>(config, "exposure_ms");

    if (camera_name == "mindvision") {
        const auto gamma = tools::read<double>(config, "gamma");
        const auto vid_pid = tools::read<std::string>(config, "vid_pid");
        camera_ = std::make_unique<MindVision>(exposure_ms, gamma, vid_pid);
    } else if (camera_name == "hikrobot") {
        const auto gain = tools::read<double>(config, "gain");
        const auto vid_pid = tools::read<std::string>(config, "vid_pid");
        camera_ = std::make_unique<HikRobot>(exposure_ms, gain, vid_pid);
    } else {
        throw std::runtime_error("Unknown camera_name: " + camera_name + "!");
    }
}

void Camera::read(cv::Mat& img, std::chrono::steady_clock::time_point& timestamp) {
    if (!camera_) {
        throw std::runtime_error("Camera is not initialized!");
    }
    camera_->read(img, timestamp);
}

} // namespace io
