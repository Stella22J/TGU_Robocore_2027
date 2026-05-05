#ifndef IO__IMU_HPP
#define IO__IMU_HPP

/**
 * @file imu.hpp
 * @brief 声明IMU串口驱动和姿态插值接口。
 *
 * IMU类负责从固定串口读取设备姿态，并提供按时间戳查询的四元数，便于相机帧和IMU姿态对齐。
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
 *
 * 使用packed保证结构体布局与设备协议一致，避免编译器padding破坏字节映射。
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
 *
 * 设备协议直接发送float二进制表示，因此保留该中间结构便于调试原始量。
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
 *
 * 该类在后台持续读取IMU姿态，并通过队列缓存带时间戳的四元数，供视觉模块按曝光时刻查询。
 */
class IMU {
  public:
    /**
     * @brief 构造IMU对象。
     *
     * 构造阶段会等待初始两帧姿态，避免第一次插值使用未初始化缓存。
     */
    IMU();

    /**
     * @brief 析构IMU对象。
     *
     * 析构时停止接收线程并关闭串口，防止串口资源泄漏。
     */
    ~IMU();

    /**
     * @brief 查询指定时间戳对应的IMU姿态。
     *
     * 通过目标时刻前后两帧四元数做slerp，使异步IMU数据更贴近相机曝光时刻。
     *
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
     *
     * 串口参数固定匹配当前IMU固件协议，启动失败时直接退出避免后续姿态不可用。
     */
    void init_serial();

    /**
     * @brief 后台IMU数据接收线程函数。
     *
     * 接收线程只负责协议解析和入队，姿态查询留给imu_at()按时间戳处理。
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
