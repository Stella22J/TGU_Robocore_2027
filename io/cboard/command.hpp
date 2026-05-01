/**
 * @file command.hpp
 * @brief 定义视觉模块发送给控制板的控制命令结构体
 * @namespace io
 */

#ifndef IO__COMMAND_HPP
#define IO__COMMAND_HPP

namespace io {
struct Command {
    bool control; // 是否启用视觉控制
    bool shoot;   // 是否请求射击
    double yaw;
    double pitch;
    double horizon_distance = 0; // 水平距离，无人机专有
};

} // namespace io

#endif // IO__COMMAND_HPP