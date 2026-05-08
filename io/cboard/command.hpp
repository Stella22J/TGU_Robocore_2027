#ifndef IO__COMMAND_HPP
#define IO__COMMAND_HPP

/**
 * @file command.hpp
 * @brief 定义视觉模块发送给控制板的控制命令。
 */

namespace io {

/**
 * @brief 视觉控制命令。
 */
struct Command {
    bool control;                 // 是否允许控制板使用视觉控制量
    bool shoot;                   // 是否请求控制板执行射击
    double yaw;                   // 目标yaw补偿量
    double pitch;                 // 目标pitch补偿量
    double horizon_distance = 0;  // 水平距离，给无人机链路保留
};

} // namespace io

#endif // IO__COMMAND_HPP
