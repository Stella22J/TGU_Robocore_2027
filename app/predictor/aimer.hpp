/**
 * @file aimer.hpp
 * @brief 弹道瞄准预测接口，把稳定目标状态转换为云台控制命令
 */

#ifndef AUTO_AIM__AIMER_HPP
#define AUTO_AIM__AIMER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>

#include "../tracker/target.hpp"
#include "io/cboard/cboard.hpp"
#include "io/cboard/command.hpp"

namespace auto_aim {

/**
 * @brief 弹道瞄准候选点，记录是否有效以及选中的装甲板空间位置
 */
struct AimPoint {
    bool valid;
    Eigen::Vector4d xyza;
};

/**
 * @brief 根据跟踪目标预测瞄准角，统一处理延迟补偿、飞行时间和弹道补偿
 *
 * 运动预测和弹道补偿放在同一层，是因为二者都依赖子弹飞行时间，并且需要迭代修正同一个击打点
 */
class Aimer {
  public:
    AimPoint debug_aim_point;
    /**
     * @brief 从TOML加载弹道偏置和延迟参数，便于实车标定后直接调整配置
     * @param config_path TOML配置文件路径，用于读取模型、阈值和运行参数
     */
    explicit Aimer(const std::string& config_path);
    /**
     * @brief 为当前跟踪目标生成瞄准命令，目标不存在时返回无控制命令
     * @param targets 候选目标列表，通常由跟踪器按当前锁定状态生成
     * @param timestamp 检测时间戳，用于补偿从图像采集到当前控制输出之间的延迟
     * @param bullet_speed 当前弹速，用于计算飞行时间和弹道补偿
     * @param to_now 是否只补偿传感器到当前时刻的延迟，调试预测链路时可使用该选项
     * @return 云台控制命令，包含是否控制、是否开火以及目标yaw、pitch
     */
    io::Command aim(std::list<Target> targets, std::chrono::steady_clock::time_point timestamp,
                    double bullet_speed, bool to_now = true);

    /**
     * @brief 根据指定发射模式生成瞄准命令，用于左右枪口或不同发射机构的偏置补偿
     * @param targets 候选目标列表，通常由跟踪器按当前锁定状态生成
     * @param timestamp 检测时间戳，用于补偿从图像采集到当前控制输出之间的延迟
     * @param bullet_speed 当前弹速，用于计算飞行时间和弹道补偿
     * @param shoot_mode 发射模式，用于选择不同摩擦轮或弹道条件下的yaw偏置
     * @param to_now 是否只补偿传感器到当前时刻的延迟，调试预测链路时可使用该选项
     * @return 云台控制命令，包含是否控制、是否开火以及目标yaw、pitch
     */
    io::Command aim(std::list<Target> targets, std::chrono::steady_clock::time_point timestamp,
                    double bullet_speed, io::ShootMode shoot_mode, bool to_now = true);

  private:
    double yaw_offset_;
    std::optional<double> left_yaw_offset_, right_yaw_offset_;
    double pitch_offset_;
    double comming_angle_;
    double leaving_angle_;
    double lock_id_ = -1;
    double high_speed_delay_time_;
    double low_speed_delay_time_;
    double decision_speed_;

    AimPoint choose_aim_point(const Target& target);
};

} // namespace auto_aim

#endif // AUTO_AIM__AIMER_HPP