#ifndef IO__IMU_HPP
#define IO__IMU_HPP

/**
 * @file imu.hpp
 * @brief IMU串口驱动接口定义。
 *
 * 该文件定义了IMU串口协议接收帧、解析后的IMU数据结构，以及DM_IMU类的对外接口。
 *
 * - 串口设备：/dev/ttyACM0
 * - 波特率：921600
 *
 * @namespace io
 */

#include <math.h>
#include "serial/serial.hpp"

#include <Eigen/Geometry>
#include <array>
#include <atomic>
#include <chrono>
#include <fstream>
#include <initializer_list>
#include <iostream>
#include <thread>

#include "tools/thread_safe_queue.hpp"

namespace io {

/**
 * @brief IMU串口原始接收帧。
 *
 * 一组完整IMU数据由三段子帧组成：
 * 1. 加速度帧：accx accy accz
 * 2. 角速度帧：gyrox gyroy gyroz
 * 3. 欧拉角帧：roll pitch yaw
 *
 * 每段子帧包含：
 * - 帧头
 * - 标志位
 * - 从机ID
 * - 寄存器地址
 * - 三个4字节浮点数据
 * - CRC16
 * - 帧尾
 */
struct __attribute__((packed)) IMU_Receive_Frame {
    // 加速度子帧
    uint8_t FrameHeader1;
    uint8_t flag1;
    uint8_t slave_id1;
    uint8_t reg_acc;
    uint32_t accx_u32;
    uint32_t accy_u32;
    uint32_t accz_u32;
    uint16_t crc1;
    uint8_t FrameEnd1;

    // 角速度子帧
    uint8_t FrameHeader2;
    uint8_t flag2;
    uint8_t slave_id2;
    uint8_t reg_gyro;
    uint32_t gyrox_u32;
    uint32_t gyroy_u32;
    uint32_t gyroz_u32;
    uint16_t crc2;
    uint8_t FrameEnd2;

    // 欧拉角子帧 r->p->y
    uint8_t FrameHeader3;
    uint8_t flag3;
    uint8_t slave_id3;
    uint8_t reg_euler;
    uint32_t roll_u32;
    uint32_t pitch_u32;
    uint32_t yaw_u32;
    uint16_t crc3;
    uint8_t FrameEnd3;
};

/**
 * @brief 解析后的IMU浮点数据。
 *
 * 原始串口帧中的uint32_t字段存放的是IEEE754 float的二进制数据。将其解析后写入该结构体。
 */
typedef struct {
    float accx; // X 轴加速度
    float accy; // Y 轴加速度
    float accz; // Z 轴加速度

    float gyrox; // X 轴角速度
    float gyroy; // Y 轴角速度
    float gyroz; // Z 轴角速度

    float roll;  // 横滚角
    float pitch; // 俯仰角
    float yaw;   // 航向角
} IMU_Data;

/**
 * @brief IMU串口读取与姿态插值类。
 *
 * 该类在构造时打开串口并启动接收线程。
 * 接收线程持续读取IMU数据，将欧拉角转换为四元数并写入线程安全队列。
 */
class IMU {
  public:
    /**
     * @brief 构造IMU对象。
     */
    IMU();

    /**
     * @brief 析构IMU对象。
     */
    ~IMU();

    /**
     * @brief 查询指定时间戳对应的IMU姿态。
     * @param timestamp 目标时间戳，使用 steady_clock。
     * @return Eigen::Quaterniond 插值后的单位四元数。
     */
    Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

  private:
    /**
     * @brief 带时间戳的姿态数据。
     */
    struct IMUData {
        Eigen::Quaterniond q;                            // IMU姿态四元数
        std::chrono::steady_clock::time_point timestamp; // 数据接收时间戳
    };

    /**
     * @brief 初始化串口参数并打开串口。
     */
    void init_serial();

    /**
     * @brief 后台IMU数据接收线程函数。
     *
     * 持续从串口读取IMU数据帧，进行帧头检查、CRC校验、数据解析，然后将欧拉角转换为四元数并推入队列。
     */
    void get_imu_data_thread();

    Serial serial_;  // 串口对象
    std::thread rec_thread_; // 后台数据接收线程

    tools::ThreadSafeQueue<IMUData> queue_; // 姿态数据线程安全队列

    // 用于imu_at()插值的目标时间前后两帧数据
    IMUData data_ahead_;
    IMUData data_behind_;

    std::atomic<bool> stop_thread_{false}; // 接收线程停止标志

    IMU_Receive_Frame receive_data{}; // 原始IMU接收帧缓存
    IMU_Data data{};                  // 解析后的IMU数据缓存
};

} // namespace io

#endif // IO__DM_IMU_HPP