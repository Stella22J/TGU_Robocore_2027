/**
 * @file camera.cpp
 * @brief 实现相机统一入口和TOML配置解析。
 *
 * 该文件集中完成配置读取、参数校验和具体相机驱动创建，避免上层模块散落硬件选择逻辑。
 */

#include "camera.hpp"

#include <stdexcept>
#include <string>

#include "hikrobot/hikrobot.hpp"
#include "mindvision/mindvision.hpp"
#include "tools/toml.hpp"

namespace io {

Camera::Camera(const std::string& config_path) {
    // 统一使用项目内tools::load/read封装，避免各模块分散处理解析失败策略
    const auto config = tools::load(config_path);
    // 读取相机类型用于选择具体驱动
    const auto camera_name = tools::read<std::string>(config, "camera_name");
    const auto exposure_ms = tools::read<double>(config, "exposure_ms");

    // 按相机类型读取专属参数并构造驱动对象

    if (camera_name == "mindvision") {
        const auto gamma = tools::read<double>(config, "gamma");
        // vid_pid用于相机异常后的USB复位
        const auto vid_pid = tools::read<std::string>(config, "vid_pid");
        camera_ = std::make_unique<MindVision>(exposure_ms, gamma, vid_pid);
    } else if (camera_name == "hikrobot") {
        const auto gain = tools::read<double>(config, "gain");
        // vid_pid用于相机异常后的USB复位
        const auto vid_pid = tools::read<std::string>(config, "vid_pid");
        camera_ = std::make_unique<HikRobot>(exposure_ms, gain, vid_pid);
    } else {
        // 配置错误直接抛出，避免系统以未知相机状态继续运行
        throw std::runtime_error("Unknown camera_name: " + camera_name + "!");
    }
}

void Camera::read(cv::Mat& img, std::chrono::steady_clock::time_point& timestamp) {
    if (!camera_) {
        throw std::runtime_error("Camera is not initialized!");
    }
    // 多态调用真实驱动的读取接口
    camera_->read(img, timestamp);
}

} // namespace io
