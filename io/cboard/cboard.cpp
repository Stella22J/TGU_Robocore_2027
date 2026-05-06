/**
 * @file cboard.cpp
 * @brief 实现控制板CAN通信封装。
 *
 * 该文件把CAN帧字节解析和控制命令打包封装在CBoard内部，减少上层模块对控制板协议细节的依赖。
 */

#include "cboard.hpp"

#include <stdexcept>
#include <string>

#include "tools/math_tools.hpp"
#include "tools/toml.hpp"

namespace io {

CBoard::CBoard(const std::string& config_path)
    : bullet_speed(0), mode(Mode::idle), shoot_mode(ShootMode::left_shoot), ft_angle(0),
      queue_(5000),
      can_(read_toml(config_path), std::bind(&CBoard::callback, this, std::placeholders::_1)) {
    // 回调可能早于构造函数结束运行，因此这里等待初始姿态，避免第一次插值读到未初始化数据
    LOG_INFO("CBoard", "Waiting for q...");
    queue_.pop(data_ahead_);
    queue_.pop(data_behind_);
    LOG_INFO("CBoard", "Opened.");
}

Eigen::Quaterniond CBoard::imu_at(std::chrono::steady_clock::time_point timestamp) {
    // 如果缓存整体落后于目标时间，先推进前一帧
    if (data_behind_.timestamp < timestamp) {
        data_ahead_ = data_behind_;
    }

    // 持续出队直到找到目标时间之后的第一帧
    while (true) {
        queue_.pop(data_behind_);
        if (data_behind_.timestamp > timestamp) {
            break;
        }
        data_ahead_ = data_behind_;
    }

    // 插值前重新归一化，避免缓存数据误差传递到姿态解算
    Eigen::Quaterniond q_a = data_ahead_.q.normalized();
    Eigen::Quaterniond q_b = data_behind_.q.normalized();

    auto t_a = data_ahead_.timestamp;
    auto t_b = data_behind_.timestamp;
    auto t_c = timestamp;

    std::chrono::duration<double> t_ab = t_b - t_a;
    std::chrono::duration<double> t_ac = t_c - t_a;

    // 用时间比例插值，避免控制板和相机频率不同造成姿态阶跃
    auto k = t_ac / t_ab;

    // 姿态插值必须保持单位四元数，否则后续旋转计算会引入尺度误差
    return q_a.slerp(k, q_b).normalized();
}

void CBoard::send(Command command) const {
    // CAN帧长度固定为8字节，按控制板协议逐字节填充
    can_frame frame;
    frame.can_id = send_canid_;
    frame.can_dlc = 8;
    frame.data[0] = command.control ? 1 : 0;
    frame.data[1] = command.shoot ? 1 : 0;
    // 角度量放大1e4后拆成高低字节，兼顾精度和协议长度
    frame.data[2] = static_cast<int16_t>(command.yaw * 1e4) >> 8;
    frame.data[3] = static_cast<int16_t>(command.yaw * 1e4);
    frame.data[4] = static_cast<int16_t>(command.pitch * 1e4) >> 8;
    frame.data[5] = static_cast<int16_t>(command.pitch * 1e4);
    frame.data[6] = static_cast<int16_t>(command.horizon_distance * 1e4) >> 8;
    frame.data[7] = static_cast<int16_t>(command.horizon_distance * 1e4);

    try {
        // 发送失败交给日志记录，不让单帧失败打断主流程
        can_.write(&frame);
    } catch (const std::exception& e) {
        LOG_WARN("CBoard", "{}", e.what());
    }
}

void CBoard::callback(const can_frame& frame) {
    // 用接收时刻作为控制板数据时间，便于和相机帧对齐
    auto timestamp = std::chrono::steady_clock::now();

    if (frame.can_id == quaternion_canid_) {
        // 四元数按高低字节还原，再缩小到协议约定尺度
        auto x = static_cast<int16_t>(frame.data[0] << 8 | frame.data[1]) / 1e4;
        auto y = static_cast<int16_t>(frame.data[2] << 8 | frame.data[3]) / 1e4;
        auto z = static_cast<int16_t>(frame.data[4] << 8 | frame.data[5]) / 1e4;
        auto w = static_cast<int16_t>(frame.data[6] << 8 | frame.data[7]) / 1e4;

        // 控制板异常帧直接丢弃，否则单位四元数假设会污染姿态插值
        if (std::abs(x * x + y * y + z * z + w * w - 1) > 1e-2) {
            LOG_WARN("CBoard", "Invalid q: {} {} {} {}", w, x, y, z);
            return;
        }

        // 有效姿态立即入队，供imu_at()按时间插值
        queue_.push({{w, x, y, z}, timestamp});
    } else if (frame.can_id == bullet_speed_canid_) {
        // 弹速和模式帧更新公开状态，供上层状态机直接读取
        bullet_speed = static_cast<int16_t>(frame.data[0] << 8 | frame.data[1]) / 1e2;
        mode = Mode(frame.data[2]);
        shoot_mode = ShootMode(frame.data[3]);
        ft_angle = static_cast<int16_t>(frame.data[4] << 8 | frame.data[5]) / 1e4;

        // 弹速日志限频，避免高频CAN帧淹没真正的异常日志
        static auto last_log_time = std::chrono::steady_clock::time_point::min();
        auto now = std::chrono::steady_clock::now();

        if (bullet_speed > 0 && tools::delta_time(now, last_log_time) >= 1.0) {
            LOG_INFO(
                "CBoard",
                "Bullet speed: {:.2f} m/s, Mode: {}, Shoot mode: {}, FT angle: {:.2f} rad",
                bullet_speed, MODES[mode], SHOOT_MODES[shoot_mode], ft_angle);
            last_log_time = now;
        }
    }
}

std::string CBoard::read_toml(const std::string& config_path) {
    // CAN协议参数集中读取，避免构造完成后才暴露配置错误
    const auto config = tools::load(config_path);

    // CANID全部来自配置，便于适配不同控制板固件
    quaternion_canid_ = tools::read<int>(config, "quaternion_canid");
    bullet_speed_canid_ = tools::read<int>(config, "bullet_speed_canid");
    send_canid_ = tools::read<int>(config, "send_canid");

    return tools::read<std::string>(config, "can_interface");
}

} // namespace io
