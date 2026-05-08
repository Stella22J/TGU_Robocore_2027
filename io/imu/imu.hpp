#ifndef IO__IMU_HPP
#define IO__IMU_HPP

/**
 * @file imu.hpp
 * @brief 声明IMU串口驱动和姿态插值接口。
 */

#include <math.h>

#include <Eigen/Geometry>
#include <array>
#include <atomic>
#include <chrono>
#include <fstream>
#include <initializer_list>
#include <iostream>
#include <thread>

#include "serial/serial.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io {

/**
 * @brief IMU串口原始接收帧。
 */
struct __attribute__((packed)) IMU_Receive_Frame {
    uint8_t FrameHeader1;
    uint8_t flag1;
    uint8_t slave_id1;
    uint8_t reg_acc;
    uint32_t accx_u32;
    uint32_t accy_u32;
    uint32_t accz_u32;
    uint16_t crc1;
    uint8_t FrameEnd1;

    uint8_t FrameHeader2;
    uint8_t flag2;
    uint8_t slave_id2;
    uint8_t reg_gyro;
    uint32_t gyrox_u32;
    uint32_t gyroy_u32;
    uint32_t gyroz_u32;
    uint16_t crc2;
    uint8_t FrameEnd2;

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
 */
typedef struct {
    float accx;
    float accy;
    float accz;
    float gyrox;
    float gyroy;
    float gyroz;
    float roll;
    float pitch;
    float yaw;
} IMU_Data;

/**
 * @brief IMU串口读取与姿态插值类。
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
     * @param timestamp 目标时间戳。
     * @return 插值得到的单位四元数。
     */
    Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

  private:
    struct IMUData {
        Eigen::Quaterniond q;                            // 使用四元数避免欧拉角奇异性
        std::chrono::steady_clock::time_point timestamp; // 使用steady_clock避免系统时间跳变影响插值
    };

    /**
     * @brief 初始化串口。
     */
    void init_serial();

    /**
     * @brief 后台IMU数据接收线程函数。
     */
    void get_imu_data_thread();

    Serial serial_;             // 用轻量串口封装替代阻塞式裸文件描述符
    std::thread rec_thread_;    // 后台读取避免阻塞视觉主循环

    tools::ThreadSafeQueue<IMUData> queue_; // 队列解耦串口接收频率和视觉查询频率

    IMUData data_ahead_;  // 缓存目标时间之前的姿态，减少重复出队
    IMUData data_behind_; // 缓存目标时间之后的姿态，供slerp插值

    std::atomic<bool> stop_thread_{false}; // 原子标志避免析构和接收线程数据竞争

    IMU_Receive_Frame receive_data{}; // 固定缓冲区减少接收循环内存分配
    IMU_Data data{};                  // 保留解析结果便于后续调试原始IMU量
};

} // namespace io

#endif // IO__IMU_HPP
