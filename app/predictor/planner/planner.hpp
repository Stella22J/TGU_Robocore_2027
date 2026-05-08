/**
 * @file planner.hpp
 * @brief TinyMPC云台规划器接口，封装MPC权重、参考轨迹和输出计划
 */

#ifndef AUTO_AIM__PLANNER_HPP
#define AUTO_AIM__PLANNER_HPP

#include <Eigen/Dense>
#include <list>
#include <optional>

#include "app/tracker/target.hpp"
#include "tinympc/tiny_api.hpp"

namespace auto_aim {
constexpr double DT = 0.01;
constexpr int HALF_HORIZON = 50;
constexpr int HORIZON = HALF_HORIZON * 2;

using Trajectory =
    Eigen::Matrix<double, 4, HORIZON>; // 轨迹状态依次为yaw、yaw速度、pitch、pitch速度

/**
 * @brief 规划后的云台状态和开火意图，包含角度、角速度、角加速度以及发射标志
 */
struct Plan {
    bool control;
    bool fire;
    float target_yaw;
    float target_pitch;
    float yaw;
    float yaw_vel;
    float yaw_acc;
    float pitch;
    float pitch_vel;
    float pitch_acc;
};

/**
 * @brief 使用TinyMPC规划yaw和pitch轨迹，让云台控制更平滑并约束动态响应
 */
class Planner {
  public:
    Eigen::Vector4d debug_xyza;
    /**
     * @brief 从TOML加载MPC权重和弹道偏置，便于现场调参而无需重新编译
     * @param config_path TOML配置文件路径，用于读取模型、阈值和运行参数
     */
    Planner(const std::string& config_path);

    /**
     * @brief 对单个明确目标生成云台计划，供控制器跟踪参考轨迹
     * @param target 待预测的目标状态，规划器会根据它生成未来参考轨迹
     * @param bullet_speed 当前弹速，用于计算飞行时间和弹道补偿
     * @return 规划后的命令状态，包含角度、角速度、角加速度和开火建议
     */
    Plan plan(Target target, double bullet_speed);
    /**
     * @brief 仅在存在目标时执行规划，避免无目标时输出虚假的控制参考
     * @param target 可选目标；为空时返回无控制计划，避免上层误用旧目标
     * @param bullet_speed 当前弹速，用于计算飞行时间和弹道补偿
     * @return 空计划或有效计划，调用方据此决定是否控制云台
     */
    Plan plan(std::optional<Target> target, double bullet_speed);

  private:
    double yaw_offset_;
    double pitch_offset_;
    double fire_thresh_;
    double low_speed_delay_time_, high_speed_delay_time_, decision_speed_;

    TinySolver* yaw_solver_;
    TinySolver* pitch_solver_;

    void setup_yaw_solver(const std::string& config_path);
    void setup_pitch_solver(const std::string& config_path);

    Eigen::Matrix<double, 2, 1> aim(const Target& target, double bullet_speed);
    Trajectory get_trajectory(Target& target, double yaw0, double bullet_speed);
};

} // namespace auto_aim

#endif // AUTO_AIM__PLANNER_HPP