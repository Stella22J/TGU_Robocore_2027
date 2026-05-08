/**
 * @file gimbal.cpp
 * @brief 实现云台串口通信、姿态缓存和控制帧发送
 */

#include "gimbal.hpp"

#include <cstdlib>
#include <exception>

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/toml.hpp"

namespace io {

Gimbal::Gimbal(const std::string& config_path) {
    auto toml = tools::load(config_path);

    com_port_ = tools::read<std::string>(toml, "com_port");

    baudrate_ = tools::read<int>(toml, "baudrate");

    serial_.recv<GimbalToVision>([this](const GimbalToVision& data) { handle_frame(data); });

    if (!serial_.open(com_port_, baudrate_)) {
        LOG_ERROR("Gimbal", "Failed to open serial:{} at {}", com_port_, baudrate_);
        std::exit(1);
    }

    thread_ = std::thread(&Gimbal::read_thread, this);

    queue_.pop();
    LOG_INFO("Gimbal", "First q received.");
}

Gimbal::~Gimbal() {
    quit_ = true;

    if (thread_.joinable())
        thread_.join();

    serial_.close();
}

GimbalMode Gimbal::mode() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return mode_;
}

GimbalState Gimbal::state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

std::string Gimbal::str(GimbalMode mode) const {
    switch (mode) {
    case GimbalMode::IDLE:
        return "IDLE";
    case GimbalMode::AUTO_AIM:
        return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF:
        return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF:
        return "BIG_BUFF";
    default:
        return "INVALID";
    }
}

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t) {
    while (true) {
        auto [q_a, t_a] = queue_.pop();
        auto [q_b, t_b] = queue_.front();

        auto t_ab = tools::delta_time(t_a, t_b);
        auto t_ac = tools::delta_time(t_a, t);

        auto k = t_ac / t_ab;
        Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();

        if (t < t_a)
            return q_c;

        if (!(t_a < t && t <= t_b))
            continue;

        return q_c;
    }
}

void Gimbal::send(io::VisionToGimbal data) {
    tx_data_.mode = data.mode;
    tx_data_.yaw = data.yaw;
    tx_data_.yaw_vel = data.yaw_vel;
    tx_data_.yaw_acc = data.yaw_acc;
    tx_data_.pitch = data.pitch;
    tx_data_.pitch_vel = data.pitch_vel;
    tx_data_.pitch_acc = data.pitch_acc;

    tx_data_.crc16 = tools::get_crc16(reinterpret_cast<uint8_t*>(&tx_data_),
                                      sizeof(tx_data_) - sizeof(tx_data_.crc16));

    if (serial_.write(reinterpret_cast<uint8_t*>(&tx_data_), sizeof(tx_data_)) !=
        sizeof(tx_data_)) {
        LOG_WARN("Gimbal", "Failed to write serial");
    }
}

void Gimbal::send(bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch,
                  float pitch_vel, float pitch_acc) {
    tx_data_.mode = control ? (fire ? 2 : 1) : 0;

    tx_data_.yaw = yaw;
    tx_data_.yaw_vel = yaw_vel;
    tx_data_.yaw_acc = yaw_acc;
    tx_data_.pitch = pitch;
    tx_data_.pitch_vel = pitch_vel;
    tx_data_.pitch_acc = pitch_acc;

    tx_data_.crc16 = tools::get_crc16(reinterpret_cast<uint8_t*>(&tx_data_),
                                      sizeof(tx_data_) - sizeof(tx_data_.crc16));

    if (serial_.write(reinterpret_cast<uint8_t*>(&tx_data_), sizeof(tx_data_)) !=
        sizeof(tx_data_)) {
        LOG_WARN("Gimbal", "Failed to write serial");
    }
}

void Gimbal::handle_frame(const GimbalToVision& data) {
    if (!tools::check_crc16(reinterpret_cast<const uint8_t*>(&data), sizeof(GimbalToVision))) {
        LOG_DEBUG("Gimbal", "CRC16 check failed.");
        return;
    }

    auto t = std::chrono::steady_clock::now();

    Eigen::Quaterniond q(data.q[0], data.q[1], data.q[2], data.q[3]);
    queue_.push({q, t});

    std::lock_guard<std::mutex> lock(mutex_);

    state_.yaw = data.yaw;
    state_.yaw_vel = data.yaw_vel;
    state_.pitch = data.pitch;
    state_.pitch_vel = data.pitch_vel;
    state_.bullet_speed = data.bullet_speed;
    state_.bullet_count = data.bullet_count;

    switch (data.mode) {
    case 0:
        mode_ = GimbalMode::IDLE;
        break;
    case 1:
        mode_ = GimbalMode::AUTO_AIM;
        break;
    case 2:
        mode_ = GimbalMode::SMALL_BUFF;
        break;
    case 3:
        mode_ = GimbalMode::BIG_BUFF;
        break;
    default:
        mode_ = GimbalMode::IDLE;
        LOG_WARN("Gimbal", "Invalid mode:{}", data.mode);
        break;
    }
}

void Gimbal::read_thread() {
    LOG_INFO("Gimbal", "read_thread started.");

    int error_count = 0;

    while (!quit_) {
        if (!serial_.is_open()) {
            error_count++;

            if (error_count > 5000) {
                error_count = 0;
                LOG_WARN("Gimbal", "Too many errors, attempting to reconnect...");
                reconnect();
            }

            continue;
        }

        serial_.spin_once();

        if (serial_.is_open()) {
            error_count = 0;
        } else {
            error_count++;
        }
    }

    LOG_INFO("Gimbal", "read_thread stopped.");
}

void Gimbal::reconnect() {
    int max_retry_count = 10;

    for (int i = 0; i < max_retry_count && !quit_; ++i) {
        LOG_WARN("Gimbal", "Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);

        serial_.close();

        try {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        } catch (...) {
            // 等待失败不影响下一次open
            LOG_WARN("Gimbal", "Waiting for serial to be reconnected...");
        }

        if (serial_.open(com_port_, baudrate_)) {
            queue_.clear();

            LOG_INFO("Gimbal", "Reconnected serial successfully.");
            break;
        }

        LOG_WARN("Gimbal", "Reconnect failed");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

} // namespace io