/**
 * @file shooter.hpp
 * @brief 开火许可判断接口，集中处理自动开火阈值和稳定性判断
 */

#ifndef AUTO_AIM__SHOOTER_HPP
#define AUTO_AIM__SHOOTER_HPP

#include <string>

#include "app/predictor/aimer.hpp"
#include "io/cboard/command.hpp"

namespace auto_aim {
/**
 * @brief 判断当前控制命令是否稳定到可以开火，避免云台追踪过程中提前发射
 */
class Shooter {
  public:
    /**
     * @brief 从TOML加载开火阈值，使近距离和远距离容差可以独立标定
     * @param config_path TOML配置文件路径，其中包含开火容差和自动开火开关
     */
    Shooter(const std::string& config_path);

    /**
     * @brief 评估当前这一帧是否满足开火条件，并更新用于稳定性判断的上一帧命令
     * @param command 最新瞄准命令，包含期望yaw、pitch和控制标志
     * @param aimer 瞄准器状态，用于确认当前击打点是否有效
     * @param targets 当前跟踪目标列表，用于判断目标距离、有效性和开火稳定性
     * @param gimbal_pos 当前云台位置，用于判断命令是否已经被机械结构跟随
     * @return 是否应该请求发弹，只有云台稳定且目标有效时才会返回true
     */
    bool shoot(const io::Command& command, const auto_aim::Aimer& aimer,
               const std::list<auto_aim::Target>& targets, const Eigen::Vector3d& gimbal_pos);

  private:
    io::Command last_command_;
    double judge_distance_;
    double first_tolerance_;
    double second_tolerance_;
    bool auto_fire_;
};
} // namespace auto_aim

#endif // AUTO_AIM__SHOOTER_HPP