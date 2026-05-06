/**
 * @file imu.cpp
 * @brief 实现IMU串口读取、协议解析和姿态插值。
 *
 * 该文件把串口协议解析限制在io模块中，上层只需要按时间戳获取四元数姿态。
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
    // 构造时打开串口并启动后台接收
    init_serial();
    rec_thread_ = std::thread(&IMU::get_imu_data_thread, this);

    // 初始化两帧缓存，保证第一次imu_at()就能做前后帧插值
    queue_.pop(data_ahead_);
    queue_.pop(data_behind_);

    LOG_INFO("IMU", "initialized");
}

IMU::~IMU() {
    // 先通知线程退出，再等待join
    stop_thread_ = true;

    if (rec_thread_.joinable()) {
        rec_thread_.join();
    }

    if (serial_.is_open()) {
        serial_.close();
    }
}

void IMU::init_serial() {
    // IMU固定使用该串口和波特率，与设备固件配置保持一致
    if (!serial_.open("/dev/ttyACM0", 921600)) {
        LOG_WARN("IMU", "failed to open serial port");
        exit(0);
    }

    // 上电后等待设备稳定，避免刚打开串口时读到半包数据
    usleep(1000000);

    LOG_INFO("IMU", "serial port opened");
}

void IMU::get_imu_data_thread() {
    while (!stop_thread_) {
        if (!serial_.is_open()) {
            LOG_WARN("IMU", "In get_imu_data_thread, imu serial port unopen");
        }

        // 先读协议固定头，失败时快速丢弃错位数据
        serial_.read(reinterpret_cast<uint8_t*>(&receive_data.FrameHeader1), 4);

        if (receive_data.FrameHeader1 == 0x55 && receive_data.flag1 == 0xAA &&
            receive_data.slave_id1 == 0x01 && receive_data.reg_acc == 0x01) {
            // 帧头对齐后读取剩余载荷
            serial_.read(reinterpret_cast<uint8_t*>(&receive_data.accx_u32), 57 - 4);

            if (tools::get_crc16(reinterpret_cast<uint8_t*>(&receive_data.FrameHeader1), 16) ==
                receive_data.crc1) {
                // 协议直接传输float二进制，按原始字节解释即可
                data.accx = *reinterpret_cast<float*>(&receive_data.accx_u32);
                data.accy = *reinterpret_cast<float*>(&receive_data.accy_u32);
                data.accz = *reinterpret_cast<float*>(&receive_data.accz_u32);
            }

            if (tools::get_crc16(reinterpret_cast<uint8_t*>(&receive_data.FrameHeader2), 16) ==
                receive_data.crc2) {
                // 角速度保留在缓存中，方便后续扩展输出
                data.gyrox = *reinterpret_cast<float*>(&receive_data.gyrox_u32);
                data.gyroy = *reinterpret_cast<float*>(&receive_data.gyroy_u32);
                data.gyroz = *reinterpret_cast<float*>(&receive_data.gyroz_u32);
            }

            if (tools::get_crc16(reinterpret_cast<uint8_t*>(&receive_data.FrameHeader3), 16) ==
                receive_data.crc3) {
                // 欧拉角用于生成统一的四元数姿态输出
                data.roll = *reinterpret_cast<float*>(&receive_data.roll_u32);
                data.pitch = *reinterpret_cast<float*>(&receive_data.pitch_u32);
                data.yaw = *reinterpret_cast<float*>(&receive_data.yaw_u32);
            }

            // 取到完整姿态后记录本机时间戳
            auto timestamp = std::chrono::steady_clock::now();

            // IMU输出为度制ZYX欧拉角，转换为上层更稳定使用的四元数
            Eigen::Quaterniond q = Eigen::AngleAxisd(data.yaw * M_PI / 180, Eigen::Vector3d::UnitZ()) *
                                   Eigen::AngleAxisd(data.pitch * M_PI / 180, Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(data.roll * M_PI / 180, Eigen::Vector3d::UnitX());

            // 归一化后再入队，避免协议抖动导致后续旋转矩阵出现尺度误差
            q.normalize();

            queue_.push({q, timestamp});
        } else {
            // 只记录错帧，接收循环继续寻找下一次对齐机会
            LOG_INFO("IMU", "failed to get correct data");
        }
    }
}

Eigen::Quaterniond IMU::imu_at(std::chrono::steady_clock::time_point timestamp) {
    // 如果查询时间超过当前缓存，推进前一帧
    if (data_behind_.timestamp < timestamp) {
        data_ahead_ = data_behind_;
    }

    // 持续取数直到找到查询时间之后的姿态
    while (true) {
        queue_.pop(data_behind_);
        if (data_behind_.timestamp > timestamp) {
            break;
        }
        data_ahead_ = data_behind_;
    }

    // 插值前归一化，保证slerp输入满足单位四元数假设
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
