/**
 * @file cboard.cpp
 * @brief 实现控制板CAN通信封装。
 *
 * 该文件把CAN帧字节解析和控制命令打包封装在CBoard内部，减少上层模块对控制板协议细节的依赖。
 */

#include "cboard.hpp"

#include <stdexcept>
#include <string>

#include "tools/math_tools.hpp"
#include "tools/toml.hpp"

namespace io {

CBoard::CBoard(const std::string& config_path)
    : bullet_speed(0), mode(Mode::idle), shoot_mode(ShootMode::left_shoot), ft_angle(0),
      queue_(5000),
      can_(read_toml(config_path), std::bind(&CBoard::callback, this, std::placeholders::_1)) {
    LOG_INFO("CBoard", "Waiting for q...");
    queue_.pop(data_ahead_);
    queue_.pop(data_behind_);
    LOG_INFO("CBoard", "Opened.");
}

Eigen::Quaterniond CBoard::imu_at(std::chrono::steady_clock::time_point timestamp) {
    if (data_behind_.timestamp < timestamp) {
        data_ahead_ = data_behind_;
    }

    while (true) {
        queue_.pop(data_behind_);
        if (data_behind_.timestamp > timestamp) {
            break;
        }
        data_ahead_ = data_behind_;
    }

    Eigen::Quaterniond q_a = data_ahead_.q.normalized();
    Eigen::Quaterniond q_b = data_behind_.q.normalized();

    auto t_a = data_ahead_.timestamp;
    auto t_b = data_behind_.timestamp;
    auto t_c = timestamp;

    std::chrono::duration<double> t_ab = t_b - t_a;
    std::chrono::duration<double> t_ac = t_c - t_a;

    auto k = t_ac / t_ab;

    return q_a.slerp(k, q_b).normalized();
}

void CBoard::send(Command command) const {
    can_frame frame;
    frame.can_id = send_canid_;
    frame.can_dlc = 8;
    frame.data[0] = command.control ? 1 : 0;
    frame.data[1] = command.shoot ? 1 : 0;
    frame.data[2] = static_cast<int16_t>(command.yaw * 1e4) >> 8;
    frame.data[3] = static_cast<int16_t>(command.yaw * 1e4);
    frame.data[4] = static_cast<int16_t>(command.pitch * 1e4) >> 8;
    frame.data[5] = static_cast<int16_t>(command.pitch * 1e4);
    frame.data[6] = static_cast<int16_t>(command.horizon_distance * 1e4) >> 8;
    frame.data[7] = static_cast<int16_t>(command.horizon_distance * 1e4);

    try {
        can_.write(&frame);
    } catch (const std::exception& e) {
        LOG_WARN("CBoard", "{}", e.what());
    }
}

void CBoard::callback(const can_frame& frame) {
    auto timestamp = std::chrono::steady_clock::now();

    if (frame.can_id == quaternion_canid_) {
        auto x = static_cast<int16_t>(frame.data[0] << 8 | frame.data[1]) / 1e4;
        auto y = static_cast<int16_t>(frame.data[2] << 8 | frame.data[3]) / 1e4;
        auto z = static_cast<int16_t>(frame.data[4] << 8 | frame.data[5]) / 1e4;
        auto w = static_cast<int16_t>(frame.data[6] << 8 | frame.data[7]) / 1e4;

        if (std::abs(x * x + y * y + z * z + w * w - 1) > 1e-2) {
            LOG_WARN("CBoard", "Invalid q: {} {} {} {}", w, x, y, z);
            return;
        }

        queue_.push({{w, x, y, z}, timestamp});
    } else if (frame.can_id == bullet_speed_canid_) {
        bullet_speed = static_cast<int16_t>(frame.data[0] << 8 | frame.data[1]) / 1e2;
        mode = Mode(frame.data[2]);
        shoot_mode = ShootMode(frame.data[3]);
        ft_angle = static_cast<int16_t>(frame.data[4] << 8 | frame.data[5]) / 1e4;

        static auto last_log_time = std::chrono::steady_clock::time_point::min();
        auto now = std::chrono::steady_clock::now();

        if (bullet_speed > 0 && tools::delta_time(now, last_log_time) >= 1.0) {
            LOG_INFO(
                "CBoard",
                "Bullet speed: {:.2f} m/s, Mode: {}, Shoot mode: {}, FT angle: {:.2f} rad",
                bullet_speed, MODES[mode], SHOOT_MODES[shoot_mode], ft_angle);
            last_log_time = now;
        }
    }
}

std::string CBoard::read_toml(const std::string& config_path) {
    const auto config = tools::load(config_path);

    quaternion_canid_ = tools::read<int>(config, "quaternion_canid");
    bullet_speed_canid_ = tools::read<int>(config, "bullet_speed_canid");
    send_canid_ = tools::read<int>(config, "send_canid");

    return tools::read<std::string>(config, "can_interface");
}

} // namespace io
