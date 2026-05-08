/**
 * @file imu.cpp
 * @brief 实现IMU串口读取、协议解析和姿态插值。
 */

#include "imu.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace io {

IMU::IMU() : queue_(5000) {
    init_serial();
    rec_thread_ = std::thread(&IMU::get_imu_data_thread, this);

    queue_.pop(data_ahead_);
    queue_.pop(data_behind_);

    LOG_INFO("IMU", "initialized");
}

IMU::~IMU() {
    stop_thread_ = true;

    if (rec_thread_.joinable()) {
        rec_thread_.join();
    }

    if (serial_.is_open()) {
        serial_.close();
    }
}

void IMU::init_serial() {
    if (!serial_.open("/dev/ttyACM0", 921600)) {
        LOG_WARN("IMU", "failed to open serial port");
        exit(0);
    }

    usleep(1000000);

    LOG_INFO("IMU", "serial port opened");
}

void IMU::get_imu_data_thread() {
    while (!stop_thread_) {
        if (!serial_.is_open()) {
            LOG_WARN("IMU", "In get_imu_data_thread, imu serial port unopen");
        }

        serial_.read(reinterpret_cast<uint8_t*>(&receive_data.FrameHeader1), 4);

        if (receive_data.FrameHeader1 == 0x55 && receive_data.flag1 == 0xAA &&
            receive_data.slave_id1 == 0x01 && receive_data.reg_acc == 0x01) {
            serial_.read(reinterpret_cast<uint8_t*>(&receive_data.accx_u32), 57 - 4);

            if (tools::get_crc16(reinterpret_cast<uint8_t*>(&receive_data.FrameHeader1), 16) ==
                receive_data.crc1) {
                data.accx = *reinterpret_cast<float*>(&receive_data.accx_u32);
                data.accy = *reinterpret_cast<float*>(&receive_data.accy_u32);
                data.accz = *reinterpret_cast<float*>(&receive_data.accz_u32);
            }

            if (tools::get_crc16(reinterpret_cast<uint8_t*>(&receive_data.FrameHeader2), 16) ==
                receive_data.crc2) {
                data.gyrox = *reinterpret_cast<float*>(&receive_data.gyrox_u32);
                data.gyroy = *reinterpret_cast<float*>(&receive_data.gyroy_u32);
                data.gyroz = *reinterpret_cast<float*>(&receive_data.gyroz_u32);
            }

            if (tools::get_crc16(reinterpret_cast<uint8_t*>(&receive_data.FrameHeader3), 16) ==
                receive_data.crc3) {
                data.roll = *reinterpret_cast<float*>(&receive_data.roll_u32);
                data.pitch = *reinterpret_cast<float*>(&receive_data.pitch_u32);
                data.yaw = *reinterpret_cast<float*>(&receive_data.yaw_u32);
            }

            auto timestamp = std::chrono::steady_clock::now();

            Eigen::Quaterniond q = Eigen::AngleAxisd(data.yaw * M_PI / 180, Eigen::Vector3d::UnitZ()) *
                                   Eigen::AngleAxisd(data.pitch * M_PI / 180, Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(data.roll * M_PI / 180, Eigen::Vector3d::UnitX());

            q.normalize();

            queue_.push({q, timestamp});
        } else {
            LOG_INFO("IMU", "failed to get correct data");
        }
    }
}

Eigen::Quaterniond IMU::imu_at(std::chrono::steady_clock::time_point timestamp) {
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

} // namespace io
