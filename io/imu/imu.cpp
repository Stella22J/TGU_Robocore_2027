/**
 * @file imu.cpp
 * @brief IMU串口驱动实现。
 *
 * 该文件实现 IMU 类，主要完成：
 * 1. 串口初始化；
 * 2. 后台线程读取IMU原始数据帧；
 * 3. CRC16 校验；
 * 4. 加速度、角速度、欧拉角解析；
 * 5. 欧拉角转四元数；
 * 6. 按时间戳缓存姿态；
 * 7. 按查询时间戳进行四元数插值。
 *
 *@namespace io
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

namespace io
{

IMU::IMU() : queue_(5000)
{
  // 初始化串口设备。
  init_serial();

  // 启动后台线程，持续从串口读取 IMU 数据。
  rec_thread_ = std::thread(&DM_IMU::get_imu_data_thread, this);

  // 预取两帧数据，用于后续 imu_at() 的时间插值。
  queue_.pop(data_ahead_);
  queue_.pop(data_behind_);

  tools::logger()->info("[IMU] initialized");
}

IMU::~IMU()
{
  // 通知后台线程退出。
  stop_thread_ = true;

  // 等待接收线程安全结束。
  if (rec_thread_.joinable()) {
    rec_thread_.join();
  }

  // 关闭串口。
  if (serial_.isOpen()) {
    serial_.close();
  }
}

void IMU::init_serial()
{
  try {
    // 串口设备路径。
    serial_.setPort("/dev/ttyACM0");

    // IMU通信波特率。
    serial_.setBaudrate(921600);

    // 串口通信参数：无流控、无校验、1位停止位、8位数据位。
    serial_.setFlowcontrol(serial::flowcontrol_none);
    serial_.setParity(serial::parity_none);
    serial_.setStopbits(serial::stopbits_one);
    serial_.setBytesize(serial::eightbits);

    // 设置串口读取超时时间。
    serial::Timeout time_out = serial::Timeout::simpleTimeout(20);
    serial_.setTimeout(time_out);

    // 打开串口。
    serial_.open();

    // 等待 IMU 或串口设备稳定。
    usleep(1000000);

    tools::logger()->info("[IMU] serial port opened");
  }

  catch (serial::IOException & e) {
    tools::logger()->warn("[IMU] failed to open serial port ");

    // 串口打开失败退出程序。
    exit(0);
  }
}

void IMU::get_imu_data_thread()
{
  while (!stop_thread_) {
    if (!serial_.isOpen()) {
      tools::logger()->warn("In get_imu_data_thread, imu serial port unopen");
    }

    // 先读取前4字节，用于判断是否为合法加速度帧起始
    serial_.read((uint8_t *)(&receive_data.FrameHeader1), 4);

    // 检查第一段加速度帧的固定帧头。
    // 合法起始为：55 AA 01 01
    if (
      receive_data.FrameHeader1 == 0x55 && receive_data.flag1 == 0xAA &&
      receive_data.slave_id1 == 0x01 && receive_data.reg_acc == 0x01)

    {
      // 完整IMU数据包长度为 57 字节,已经读取4字节，继续读取剩余53字节。
      serial_.read((uint8_t *)(&receive_data.accx_u32), 57 - 4);

      // 校验并解析加速度帧(CRC校验范围为每段子帧前16字节)
      if (tools::get_crc16((uint8_t *)(&receive_data.FrameHeader1), 16) == receive_data.crc1) {
        data.accx = *((float *)(&receive_data.accx_u32));
        data.accy = *((float *)(&receive_data.accy_u32));
        data.accz = *((float *)(&receive_data.accz_u32));
      }

      // 校验并解析角速度帧。
      if (tools::get_crc16((uint8_t *)(&receive_data.FrameHeader2), 16) == receive_data.crc2) {
        data.gyrox = *((float *)(&receive_data.gyrox_u32));
        data.gyroy = *((float *)(&receive_data.gyroy_u32));
        data.gyroz = *((float *)(&receive_data.gyroz_u32));
      }

      // 校验并解析欧拉角帧。
      if (tools::get_crc16((uint8_t *)(&receive_data.FrameHeader3), 16) == receive_data.crc3) {
        data.roll = *((float *)(&receive_data.roll_u32));
        data.pitch = *((float *)(&receive_data.pitch_u32));
        data.yaw = *((float *)(&receive_data.yaw_u32));

        // 调试姿态角
        // tools::logger()->debug(
        //   "yaw: {:.2f}, pitch: {:.2f}, roll: {:.2f}", static_cast<double>(data.yaw),
        //   static_cast<double>(data.pitch), static_cast<double>(data.roll));
      }

      // 使用当前接收时刻作为该 IMU 姿态的时间戳。
      auto timestamp = std::chrono::steady_clock::now();

      // 将Z-Y-X顺序欧拉角转换为四元数：
      Eigen::Quaterniond q = Eigen::AngleAxisd(data.yaw * M_PI / 180, Eigen::Vector3d::UnitZ()) *
                             Eigen::AngleAxisd(data.pitch * M_PI / 180, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(data.roll * M_PI / 180, Eigen::Vector3d::UnitX());

      // 保证四元数为单位四元数，避免后续插值误差累积。
      q.normalize();

      // 将姿态和时间戳写入线程安全队列，供 imu_at() 查询。
      queue_.push({q, timestamp});
    } else {
      // 帧头不匹配，说明当前串口数据没有对齐到合法数据包起始位置。
      tools::logger()->info("[IMU] failed to get correct data");
    }
  }
}

Eigen::Quaterniond DM_IMU::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  // 如果当前后一帧仍然早于目标时间，则推进前一帧。
  if (data_behind_.timestamp < timestamp) data_ahead_ = data_behind_;

  // 从队列中持续取数据，直到找到第一帧晚于目标时间的姿态。
  while (true) {
    queue_.pop(data_behind_);
    if (data_behind_.timestamp > timestamp) break;
    data_ahead_ = data_behind_;
  }

  // 目标时间前后的两个姿态四元数。
  Eigen::Quaterniond q_a = data_ahead_.q.normalized();
  Eigen::Quaterniond q_b = data_behind_.q.normalized();

  auto t_a = data_ahead_.timestamp;
  auto t_b = data_behind_.timestamp;
  auto t_c = timestamp;

  // 计算目标时间在两帧之间的比例。
  std::chrono::duration<double> t_ab = t_b - t_a;
  std::chrono::duration<double> t_ac = t_c - t_a;

  auto k = t_ac / t_ab;

  // 使用四元数球面线性插值，得到目标时间的姿态。
  Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();

  return q_c;
}

}  // namespace io