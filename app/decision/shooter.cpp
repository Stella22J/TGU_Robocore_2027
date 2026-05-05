/**
 * @file shooter.cpp
 * @brief 开火许可判断实现，保证云台稳定且瞄准点有效时才允许发射
 */

#include "shooter.hpp"

#include "io/cboard/command.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/toml.hpp"

namespace auto_aim {
Shooter::Shooter(const std::string& config_path) : last_command_{false, false, 0, 0} {
    auto config = tools::load(config_path);
    first_tolerance_ = tools::read<double>(config, "first_tolerance") /
                       57.3; // 配置使用角度，控制和预测统一使用弧度
    second_tolerance_ = tools::read<double>(config, "second_tolerance") /
                        57.3; // 配置使用角度，控制和预测统一使用弧度
    judge_distance_ = tools::read<double>(config, "judge_distance");
    auto_fire_ = tools::read<bool>(config, "auto_fire");
}

bool Shooter::shoot(const io::Command& command, const auto_aim::Aimer& aimer,
                    const std::list<auto_aim::Target>& targets, const Eigen::Vector3d& gimbal_pos) {
    if (!command.control || targets.empty() || !auto_fire_)
        return false;

    auto target_x = targets.front().ekf_x()[0];
    auto target_y = targets.front().ekf_x()[2];
    auto tolerance = std::sqrt(tools::square(target_x) + tools::square(target_y)) > judge_distance_
                         ? second_tolerance_
                         : first_tolerance_;
    // 调试几何筛选过程，正式运行时避免高频日志影响实时性"d(command.yaw) is {:.4f}",
    // std::abs(last_command_.yaw - command.yaw));
    if (std::abs(last_command_.yaw - command.yaw) <
            tolerance * 2 && // 此时认为command突变不应该射击
        std::abs(gimbal_pos[0] - last_command_.yaw) < tolerance && // 应该减去上一次command的yaw值
        aimer.debug_aim_point.valid) {
        last_command_ = command;
        return true;
    }

    last_command_ = command;
    return false;
}

} // namespace auto_aim