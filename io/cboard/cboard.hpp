/**
 * @file cboard.hpp
 * @brief 封装控制板CAN通信
 *
 * CBoard基于SocketCAN实现控制板通信
 * 主要负责接收IMU四元数、弹速、工作模式和射击模式,向控制板发送视觉控制命令
 *
 * @namespace io
 */

#ifndef IO__CBOARD_HPP
#define IO__CBOARD_HPP

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include "io/command.hpp"
#include "io/socketcan.hpp"
#include "tools/logger.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io {

// 控制板工作模式
enum Mode { idle, auto_aim, small_buff, big_buff, outpost };

// 控制板工作模式字符串
const std::vector<std::string> MODES = {"idle", "auto_aim", "small_buff", "big_buff", "outpost"};

// 哨兵射击模式
enum ShootMode { left_shoot, right_shoot, both_shoot };

// 哨兵射击模式字符串
const std::vector<std::string> SHOOT_MODES = {"left_shoot", "right_shoot", "both_shoot"};

/**
 * @brief 控制板CAN通信类
 *
 * 通过quaternion_canid_接收IMU四元数
 * 通过bullet_speed_canid_接收弹速、模式、射击模式和飞塔角度
 * 通过send_canid_发送视觉控制命令
 */
class CBoard {
  public:
    // 当前弹速
    double bullet_speed;

    // 当前工作模式
    Mode mode;

    // 当前射击模式
    ShootMode shoot_mode;

    // 飞塔角度,无人机专有字段
    double ft_angle;

    /**
     * @brief 构造控制板通信对象
     * @param config_path TOML配置文件路径
     */
    explicit CBoard(const std::string& config_path);

    /**
     * @brief 查询指定时间点的IMU姿态
     * @param timestamp 目标时间戳
     * @return 插值得到的IMU四元数
     */
    Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

    /**
     * @brief 发送视觉控制命令
     * @param command 视觉控制命令
     */
    void send(Command command) const;

  private:
    // IMU姿态数据
    struct IMUData {
        // IMU四元数
        Eigen::Quaterniond q;

        // 接收该四元数时的本机时间戳
        std::chrono::steady_clock::time_point timestamp;
    };

    // IMU数据队列,必须在can_之前初始化
    tools::ThreadSafeQueue<IMUData> queue_;

    // SocketCAN底层通信对象
    SocketCAN can_;

    // 插值时位于目标时间之前的IMU数据
    IMUData data_ahead_;

    // 插值时位于目标时间之后的IMU数据
    IMUData data_behind_;

    // IMU四元数CANID
    int quaternion_canid_;

    // 弹速和模式CANID
    int bullet_speed_canid_;

    // 视觉控制命令发送CANID
    int send_canid_;

    /**
     * @brief CAN帧接收回调
     * @param frame 收到的CAN帧
     */
    void callback(const can_frame& frame);

    /**
     * @brief 读取TOML配置
     * @param config_path TOML配置文件路径
     * @return CAN接口名
     */
    std::string read_toml(const std::string& config_path);
};

} // namespace io

#endif // IO__CBOARD_HPP