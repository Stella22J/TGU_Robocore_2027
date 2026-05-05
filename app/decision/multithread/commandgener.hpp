/**
 * @file commandgener.hpp
 * @brief 高频命令生成接口，解耦感知帧率和云台控制发送频率
 */

#ifndef AUTO_AIM_MULTITHREAD__HPP
#define AUTO_AIM_MULTITHREAD__HPP

#include <optional>

#include "app/decision/decider.hpp"
#include "app/decision/shooter.hpp"
#include "app/tracker/tracker.hpp"
#include "io/cboard/cboard.hpp"
#include "tools/plotter.hpp"

namespace auto_aim {
namespace multithread {

/**
 * @brief 高频生成并发送瞄准命令，避免视觉帧率较低时下位机控制输入断续
 *
 * 命令输出与检测帧率解耦，是为了在检测较慢时仍向控制器发送高频预测结果
 */
class CommandGener {
  public:
    CommandGener(auto_aim::Shooter& shooter, auto_aim::Aimer& aimer, io::CBoard& cboard,
                 tools::Plotter& plotter, bool debug = false);

    /**
     * @brief 停止命令生成线程，确保退出时通信板和共享数据不会被继续访问
     */
    ~CommandGener();

    /**
     * @brief 发布最新感知状态给生成线程，使控制命令始终基于最近一次目标信息
     * @param targets 最新跟踪目标列表，命令线程会基于它持续预测
     * @param t 最新检测结果的时间戳，用于补偿传感器到当前时刻的延迟
     * @param bullet_speed 当前弹速，用于计算飞行时间和弹道补偿
     * @param gimbal_pos 当前云台位置，用于判断命令是否已经被机械结构跟随
     */
    void push(const std::list<auto_aim::Target>& targets,
              const std::chrono::steady_clock::time_point& t, double bullet_speed,
              const Eigen::Vector3d& gimbal_pos);

  private:
    struct Input {
        std::list<auto_aim::Target> targets_;
        std::chrono::steady_clock::time_point t;
        // 预留外部决策回调，当前线程直接使用Aimer和Shooter生成命令
        double bullet_speed;
        Eigen::Vector3d gimbal_pos;
    };

    io::CBoard& cboard_;
    auto_aim::Shooter& shooter_;
    auto_aim::Aimer& aimer_;
    tools::Plotter& plotter_;

    std::optional<Input> latest_;
    std::mutex mtx_;
    std::condition_variable cv_;
    std::thread thread_;
    bool stop_, debug_;

    void generate_command();
};

} // namespace multithread

} // namespace auto_aim

#endif // AUTO_AIM_MULTITHREAD__HPP