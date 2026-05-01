/**
 * @file camera.cpp
 * @brief 相机统一入口与 TOML 配置解析实现。
 *
 * 本文件实现Camera类。
 *
 * 程序主要流程：
 * 1. 读取TOML配置文件
 * 2. 解析camera_name和exposure_ms
 * 3. 根据camera_name判断具体相机类型
 * 4. 如果是mindvision，则读取gamma和vid_pid，并创建MindVision；如果是hikrobot，则读取gain和vid_pid，并创建HikRobot
 * 5. 上层调用Camera::read()时，将读取请求转发给具体相机对象
 *
 *@namespace io
 */

#include "camera.hpp"

#include <stdexcept>
#include <string>

#include <toml++/toml.hpp>

#include "hikrobot/hikrobot.hpp"
#include "mindvision/mindvision.hpp"

namespace io {
namespace {

/**
 * @brief 从TOML表中读取指定key的值。
 *
 * 如果 key 不存在，或类型不匹配，则抛出 std::runtime_error。
 *
 * @tparam T 目标类型
 * @param table TOML 根表
 * @param key 配置项名称
 * @return T 配置项对应的值
 */
template <typename T> T read_toml_value(const toml::table& table, const std::string& key) {
    auto value = table[key].value<T>();

    if (!value) {
        throw std::runtime_error("Missing or invalid TOML field: " + key);
    }

    return *value;
}

} // namespace

Camera::Camera(const std::string& config_path) {
    toml::table config;

    try {
        // 解析TOML配置文件。
        config = toml::parse_file(config_path);
    } catch (const toml::parse_error& e) {
        throw std::runtime_error("Failed to parse camera config: " + config_path +
                                 ", error: " + std::string(e.description()));
    }

    // 读取所有相机共有配置
    const auto camera_name = read_toml_value<std::string>(config, "camera_name");
    const auto exposure_ms = read_toml_value<double>(config, "exposure_ms");

    if (camera_name == "mindvision") {
        // MindVision 相机特有参数
        const auto gamma = read_toml_value<double>(config, "gamma");
        const auto vid_pid = read_toml_value<std::string>(config, "vid_pid");

        camera_ = std::make_unique<MindVision>(exposure_ms, gamma, vid_pid);
    }

    else if (camera_name == "hikrobot") {
        // HikRobot 相机特有参数
        const auto gain = read_toml_value<double>(config, "gain");
        const auto vid_pid = read_toml_value<std::string>(config, "vid_pid");

        camera_ = std::make_unique<HikRobot>(exposure_ms, gain, vid_pid);
    }

    else {
        throw std::runtime_error("Unknown camera_name: " + camera_name + "!");
    }
}

void Camera::read(cv::Mat& img, std::chrono::steady_clock::time_point& timestamp) {
    camera_->read(img, timestamp);
}

} // namespace io