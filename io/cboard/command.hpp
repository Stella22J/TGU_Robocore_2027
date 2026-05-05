#ifndef IO__COMMAND_HPP
#define IO__COMMAND_HPP

/**
 * @file command.hpp
 * @brief 定义视觉模块发送给控制板的控制命令。
 *
 * 命令字段保持为简单POD结构，便于在控制链路中直接构造、复制和序列化。
 */

namespace io {

/**
 * @brief 视觉控制命令。
 *
 * 该结构体只表达控制意图，实际量纲映射由发送端CAN封包逻辑统一处理。
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
