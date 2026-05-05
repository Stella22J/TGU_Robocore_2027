#ifndef IO__CBOARD_HPP
#define IO__CBOARD_HPP

/**
 * @file cboard.hpp
 * @brief 声明控制板CAN通信封装。
 *
 * CBoard把控制板协议集中在io模块中，上层只需要读取姿态和发送控制命令，不需要直接处理CANID和字节打包。
 */

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include "command.hpp"
#include "socketcan.hpp"
#include "tools/logger.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io {

// 与控制板协议枚举保持同序，避免日志和状态分支错位
enum Mode { idle, auto_aim, small_buff, big_buff, outpost };

// 使用表驱动打印模式，避免多个switch分支重复维护
const std::vector<std::string> MODES = {"idle", "auto_aim", "small_buff", "big_buff", "outpost"};

// 与控制板协议枚举保持同序，CAN字节可直接映射
enum ShootMode { left_shoot, right_shoot, both_shoot };

// 使用表驱动打印射击模式，便于调试CAN协议
const std::vector<std::string> SHOOT_MODES = {"left_shoot", "right_shoot", "both_shoot"};

/**
 * @brief 控制板CAN通信类。
 *
 * 该类负责接收控制板广播的IMU四元数、弹速和模式信息，并按协议发送视觉控制命令。
 */
class CBoard {
  public:
    double bullet_speed;      ///< 当前弹速，供预测模块避免重复解析CAN帧
    Mode mode;                ///< 当前工作模式，供上层状态机直接判断
    ShootMode shoot_mode;     ///< 当前射击模式，供瞄准策略选择射击侧
    double ft_angle;          ///< 飞塔角度，给无人机链路保留

    /**
     * @brief 构造控制板通信对象。
     *
     * 构造时会读取TOML配置并启动SocketCAN接收线程，因此配置错误应尽早暴露。
     *
     * @param config_path TOML配置文件路径。
     */
    explicit CBoard(const std::string& config_path);

    /**
     * @brief 查询指定时间点的IMU姿态。
     *
     * 控制板IMU和相机不同步，使用时间戳插值可以让视觉处理拿到更接近曝光时刻的姿态。
     *
     * @param timestamp 目标时间戳。
     * @return 插值得到的IMU四元数。
     */
    Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

    /**
     * @brief 发送视觉控制命令。
     *
     * @param command 视觉控制命令。
     */
    void send(Command command) const;

  private:
    struct IMUData {
        Eigen::Quaterniond q;                            // 缓存四元数，避免上层接触CAN字节序
        std::chrono::steady_clock::time_point timestamp; // 使用本机时间戳支持跨传感器对齐
    };

    tools::ThreadSafeQueue<IMUData> queue_; // 必须早于can_初始化，因为回调可能在构造过程中到达
    SocketCAN can_;                         // 放在queue_之后以保证回调依赖对象已构造

    IMUData data_ahead_;  // 缓存目标时间之前的数据，减少重复出队
    IMUData data_behind_; // 缓存目标时间之后的数据，供slerp插值

    int quaternion_canid_;     // 从配置读取，方便不同控制板协议复用同一代码
    int bullet_speed_canid_;   // 从配置读取，避免CANID硬编码到源码
    int send_canid_;           // 从配置读取，便于现场调试时只改配置

    /**
     * @brief 处理收到的CAN帧。
     *
     * 回调中只做协议解析和轻量入队，避免阻塞SocketCAN接收线程。
     *
     * @param frame 收到的CAN帧。
     */
    void callback(const can_frame& frame);

    /**
     * @brief 读取TOML配置并返回CAN接口名。
     *
     * 该函数顺带填充CANID成员，使构造函数可以在初始化can_前完成协议参数加载。
     *
     * @param config_path TOML配置文件路径。
     * @return CAN接口名。
     */
    std::string read_toml(const std::string& config_path);
};

} // namespace io

#endif // IO__CBOARD_HPP
