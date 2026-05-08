/**
 * @file tracker.hpp
 * @brief 装甲目标跟踪器接口，封装目标选择、状态机和全向感知切换逻辑
 */

#ifndef AUTO_AIM__TRACKER_HPP
#define AUTO_AIM__TRACKER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <string>

#include "../auto_aim/armor.hpp"
#include "../auto_aim/solver.hpp"
#include "app/auto_aim/perceptron.hpp"
#include "target.hpp"
#include "tools/thread_safe_queue.hpp"

namespace auto_aim {
/**
 * @brief 跨帧维护目标锁定状态，减少单帧误检对云台控制的影响
 */
class Tracker {
  public:
    /**
     * @brief 从TOML加载跟踪阈值并绑定位姿解算器，使检测结果能转换为目标状态
     * @param config_path TOML配置文件路径，用于读取模型、阈值和运行参数
     * @param solver 位姿解算器，用于把装甲板像素角点更新为三维观测
     */
    Tracker(const std::string& config_path, Solver& solver);

    /**
     * @brief 返回当前跟踪状态名称，便于日志和上位机显示状态机行为
     * @return 当前跟踪状态字符串，用于日志和调试界面显示
     */
    std::string state() const;

    /**
     * @brief 使用主相机检测结果更新锁定目标，这是正常自瞄闭环的主要入口
     * @param armors 当前帧装甲板观测列表，函数会在原列表中写入三维位姿并进行过滤
     * @param t 当前检测结果对应的时间戳，用于计算预测和更新的时间间隔
     * @param use_enemy_color 是否按配置中的敌方颜色过滤，调试阶段可关闭以观察所有目标
     * @return 当前有效目标列表，通常包含正在锁定或临时丢失但仍可预测的目标
     */
    std::list<Target> track(std::list<Armor>& armors, std::chrono::steady_clock::time_point t,
                            bool use_enemy_color = true);

    /**
     * @brief 结合全向感知切换提示更新目标锁定，用于从当前目标转向更高优先级目标
     * @param detection_queue 全向感知模块输出的检测结果，用于发现主相机视野外的高优先级目标
     * @param armors 主相机当前帧装甲板观测，函数会在原列表中写入解算结果并参与目标切换判断
     * @param t 当前检测结果对应的时间戳，用于计算预测和更新的时间间隔
     * @param use_enemy_color 是否按配置中的敌方颜色过滤，调试阶段可关闭以观察所有目标
     * @return 切换目标提示和当前有效目标列表，提示用于引导云台转向新目标
     */
    std::tuple<omniperception::DetectionResult, std::list<Target>>
    track(const std::vector<omniperception::DetectionResult>& detection_queue,
          std::list<Armor>& armors, std::chrono::steady_clock::time_point t,
          bool use_enemy_color = true);

  private:
    Solver& solver_;
    Color enemy_color_;
    int min_detect_count_;
    int max_temp_lost_count_;
    int detect_count_;
    int temp_lost_count_;
    int outpost_max_temp_lost_count_;
    int normal_temp_lost_count_;
    std::string state_, pre_state_;
    Target target_;
    std::chrono::steady_clock::time_point last_timestamp_;
    ArmorPriority omni_target_priority_;

    void state_machine(bool found);

    bool set_target(std::list<Armor>& armors, std::chrono::steady_clock::time_point t);

    bool update_target(std::list<Armor>& armors, std::chrono::steady_clock::time_point t);
};

} // namespace auto_aim

#endif // AUTO_AIM__TRACKER_HPP