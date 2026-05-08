/**
 * @file gimbal.hpp
 * @brief 定义云台串口通信接口与上下位机数据帧
 */

#ifndef IO__GIMBAL_HPP
#define IO__GIMBAL_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>

#include "io/serial/serial.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io {

/**
 * @brief 云台发送给视觉模块的数据帧
 */
struct __attribute__((packed)) GimbalToVision {
    // 固定帧头用于在串口流中重新对齐数据帧
    uint8_t head[2] = {'S', 'P'};

    // 模式由下位机统一编码,避免上下位机维护多套字符串协议
    uint8_t mode;

    // 四元数使用wxyz顺序,与Eigen::Quaterniond构造顺序一致
    float q[4];

    // 云台状态用于预测和控制闭环
    float yaw;
    float yaw_vel;
    float pitch;
    float pitch_vel;

    // 弹速和弹丸计数来自控制链路,视觉端只消费不计算
    float bullet_speed;
    uint16_t bullet_count;

    // CRC放在帧尾,便于对前面全部载荷做完整性校验
    uint16_t crc16;
};

// 单帧控制在64字节内,避免底层串口读取缓存过大
static_assert(sizeof(GimbalToVision) <= 64);

/**
 * @brief 视觉模块发送给云台的数据帧
 */
struct __attribute__((packed)) VisionToGimbal {
    // 与接收帧使用相同帧头,便于下位机复用帧同步逻辑
    uint8_t head[2] = {'S', 'P'};

    // 0表示不控制,1表示控制不开火,2表示控制并开火
    uint8_t mode;

    // yaw和pitch同时发送位置、速度、加速度,用于下位机做平滑控制
    float yaw;
    float yaw_vel;
    float yaw_acc;
    float pitch;
    float pitch_vel;
    float pitch_acc;

    // CRC放在最后,发送前根据完整载荷重新计算
    uint16_t crc16;
};

// 单帧控制在64字节内,保证协议适合低延迟串口通信
static_assert(sizeof(VisionToGimbal) <= 64);

/**
 * @brief 云台当前工作模式
 */
enum class GimbalMode { IDLE, AUTO_AIM, SMALL_BUFF, BIG_BUFF };

/**
 * @brief 云台状态快照
 */
struct GimbalState {
    float yaw;
    float yaw_vel;
    float pitch;
    float pitch_vel;
    float bullet_speed;
    uint16_t bullet_count;
};

/**
 * @brief 云台串口通信封装
 */
class Gimbal {
  public:
    /**
     * @brief 创建云台通信对象
     * @param config_path TOML配置文件路径
     */
    explicit Gimbal(const std::string& config_path);

    /**
     * @brief 销毁云台通信对象
     */
    ~Gimbal();

    /**
     * @brief 获取当前云台模式
     * @return 最近一次通过串口接收到的云台模式
     */
    GimbalMode mode() const;

    /**
     * @brief 获取当前云台状态
     * @return 最近一次通过串口接收到的云台状态快照
     */
    GimbalState state() const;

    /**
     * @brief 将云台模式转换为字符串
     * @param mode 待转换的云台模式
     * @return 模式字符串
     */
    std::string str(GimbalMode mode) const;

    /**
     * @brief 查询指定时间点的云台姿态
     * @param t 目标时间点
     * @return 插值得到的四元数姿态
     */
    Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);

    /**
     * @brief 发送视觉控制量
     * @param control 是否启用视觉控制
     * @param fire 是否请求开火
     * @param yaw yaw位置控制量
     * @param yaw_vel yaw速度前馈
     * @param yaw_acc yaw加速度前馈
     * @param pitch pitch位置控制量
     * @param pitch_vel pitch速度前馈
     * @param pitch_acc pitch加速度前馈
     */
    void send(bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch,
              float pitch_vel, float pitch_acc);

    /**
     * @brief 发送完整视觉到云台数据帧
     * @param data 已填充的视觉控制帧
     */
    void send(io::VisionToGimbal data);

  private:
    // 当前项目的Serial已经提供结构体帧头解析,这里不再使用旧serial::Serial
    Serial serial_;

    // 保存配置,重连时需要复用相同串口参数
    std::string com_port_;
    int baudrate_ = 115200;

    // 接收线程独立运行,避免主视觉循环阻塞在串口读操作上
    std::thread thread_;
    std::atomic<bool> quit_ = false;

    // 状态会被接收线程和业务线程同时访问,因此需要互斥保护
    mutable std::mutex mutex_;

    // 发送缓冲区复用,减少循环中的临时对象构造
    VisionToGimbal tx_data_;

    // mode_和state_保存最近一次有效帧,外部读取时无需直接解析协议
    GimbalMode mode_ = GimbalMode::IDLE;
    GimbalState state_{};

    // 姿态队列用于按图像时间戳回溯插值
    tools::ThreadSafeQueue<std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>>
        queue_{1000};

    // Serial通过recv回调给出完整结构体帧,这里负责CRC校验和业务状态更新
    void handle_frame(const GimbalToVision& data);

    // 后台线程持续驱动Serial读取,因为当前Serial采用spin_once()轮询回调模型
    void read_thread();

    // 串口错误过多时重连,避免偶发断线导致整个进程退出
    void reconnect();
};

} // namespace io

#endif // IO__GIMBAL_HPP