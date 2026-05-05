/**
 * @file decider.hpp
 * @brief 全向感知目标决策接口，负责目标过滤、优先级排序和粗略角度计算
 *
 * 该文件随所属模块放置，便于按功能维护，同时保持跨模块接口清晰
 */

#ifndef OMNIPERCEPTION__DECIDER_HPP
#define OMNIPERCEPTION__DECIDER_HPP

#include <Eigen/Dense> // 必须在opencv2/core/eigen.hpp上面
#include <iostream>
#include <list>
#include <unordered_map>

#include "app/auto_aim/armor.hpp"
#include "app/auto_aim/detection.hpp"
#include "app/auto_aim/yolo.hpp"
#include "app/tracker/target.hpp"
#include "io/camera.hpp"
#include "io/command.hpp"
#include "io/usbcamera/usbcamera.hpp"

namespace omniperception {
/**
 * @brief 选择全向感知目标并计算粗略云台角度，用于主相机切换到新方向
 *
 * 该层与精确自瞄分离，是为了在主相机完成PnP锁定前，就能根据全向感知结果提前引导云台切换目标
 */
class Decider {
  public:
    /**
     * @brief 从TOML加载相机视场角和策略参数，使安装角和目标优先级可现场调整
     * @param config_path TOML配置文件路径，用于读取模型、阈值和运行参数
     */
    Decider(const std::string& config_path);

    /**
     * @brief 轮询侧后相机并生成粗瞄命令，在主相机尚未看到目标前完成转向引导
     * @param yolo 轮询路径共享的检测器实例，用于减少模型重复加载开销
     * @param gimbal_pos 当前云台姿态，用于把相机相对偏角转换为云台目标角
     * @param usbcam1 第一路侧向相机，用于覆盖主相机之外的视野
     * @param usbcam2 第二路侧向相机，用于覆盖另一侧视野
     * @param back_cammera 后向相机，参数名保留原拼写以兼容已有代码
     * @return 粗略云台控制命令，主要用于发现和切换目标而非精确击打
     */
    io::Command decide(auto_aim::YOLO& yolo, const Eigen::Vector3d& gimbal_pos,
                       io::USBCamera& usbcam1, io::USBCamera& usbcam2, io::Camera& back_cammera);

    /**
     * @brief 只使用后置相机执行粗决策，适合调试或简化全向感知输入
     * @param yolo 轮询路径共享的检测器实例，用于减少模型重复加载开销
     * @param gimbal_pos 当前云台姿态，用于把相机相对偏角转换为云台目标角
     * @param back_cammera 后向相机，参数名保留原拼写以兼容已有代码
     * @return 粗略云台控制命令，主要用于发现和切换目标而非精确击打
     */
    io::Command decide(auto_aim::YOLO& yolo, const Eigen::Vector3d& gimbal_pos,
                       io::Camera& back_cammera);

    /**
     * @brief 把已经排序的并行检测结果转换为粗瞄命令，减少主循环中的决策逻辑
     * @param detection_queue 感知线程输出的检测结果队列，可能包含多个相机的候选目标
     * @return 粗略云台控制命令，主要用于发现和切换目标而非精确击打
     */
    io::Command decide(const std::vector<DetectionResult>& detection_queue);

    /**
     * @brief 计算相机相对目标的yaw和pitch偏移，结合安装角得到云台粗略转向量
     * @param armors 候选装甲板列表，调用前通常已经按决策优先级排序
     * @param camera 相机名称，用于选择对应安装偏置和视场角模型
     * @return 以角度表示的yaw和pitch偏差，调用方会按通信协议继续转换
     */
    Eigen::Vector2d delta_angle(const std::list<auto_aim::Armor>& armors,
                                const std::string& camera);

    /**
     * @brief 移除不应攻击的目标，例如己方、无敌、前哨站或赛季不存在的编号
     * @param armors 待处理的装甲板列表，函数会在原列表上过滤或重排
     * @return 是否已经没有可用装甲板，用于快速返回无目标命令
     */
    bool armor_filter(std::list<auto_aim::Armor>& armors);

    /**
     * @brief 应用配置中的目标优先级模式，让同一套识别结果适配不同战术需求
     * @param armors 待处理的装甲板列表，函数会在原列表上过滤或重排
     */
    void set_priority(std::list<auto_aim::Armor>& armors);
    // 对队列中的每一个DetectionResult进行过滤，同时将DetectionResult排序
    /**
     * @brief 过滤并排序并行检测结果，使最高优先级目标排在决策队列前部
     * @param detection_queue 检测队列快照，函数会过滤空结果并按最高优先级排序
     */
    void sort(std::vector<DetectionResult>& detection_queue);

    /**
     * @brief 提取目标信息给上层模块通信，避免直接暴露完整Armor内部结构
     * @param armors 当前帧装甲板观测，用于从识别结果中提取目标信息
     * @param targets 当前跟踪器目标，用于和观测装甲板匹配身份
     * @return 打包后的目标信息向量，用于和通信协议或上层策略对接
     */
    Eigen::Vector4d get_target_info(const std::list<auto_aim::Armor>& armors,
                                    const std::list<auto_aim::Target>& targets);

    /**
     * @brief 更新无敌目标编号列表，避免自瞄系统攻击当前不可造成伤害的敌人
     * @param invincible_enemy_ids 裁判系统或策略层下发的无敌目标编号，使用通信协议中的编号约定
     */
    void get_invincible_armor(const std::vector<int8_t>& invincible_enemy_ids);

    /**
     * @brief 只保留外部指定的自瞄目标，用于导航或策略模块强制锁定目标
     * @param armors 待处理的装甲板列表，函数会在原列表上过滤或重排
     * @param auto_aim_target 策略层允许自瞄攻击的目标编号，使用通信协议中的编号约定
     */
    void get_auto_aim_target(std::list<auto_aim::Armor>& armors,
                             const std::vector<int8_t>& auto_aim_target);

  private:
    int img_width_;
    int img_height_;
    double fov_h_, new_fov_h_;
    double fov_v_, new_fov_v_;
    int mode_;
    int count_;

    auto_aim::Color enemy_color_;
    auto_aim::YOLO detector_;
    std::vector<auto_aim::ArmorName> invincible_armor_; // 无敌状态机器人编号,英雄为1，哨兵为6

    // 定义ArmorName到ArmorPriority的映射类型
    using PriorityMap = std::unordered_map<auto_aim::ArmorName, auto_aim::ArmorPriority>;

    const PriorityMap mode1 = {{auto_aim::ArmorName::one, auto_aim::ArmorPriority::second},
                               {auto_aim::ArmorName::two, auto_aim::ArmorPriority::forth},
                               {auto_aim::ArmorName::three, auto_aim::ArmorPriority::first},
                               {auto_aim::ArmorName::four, auto_aim::ArmorPriority::first},
                               {auto_aim::ArmorName::five, auto_aim::ArmorPriority::third},
                               {auto_aim::ArmorName::sentry, auto_aim::ArmorPriority::third},
                               {auto_aim::ArmorName::outpost, auto_aim::ArmorPriority::fifth},
                               {auto_aim::ArmorName::base, auto_aim::ArmorPriority::fifth},
                               {auto_aim::ArmorName::not_armor, auto_aim::ArmorPriority::fifth}};

    const PriorityMap mode2 = {{auto_aim::ArmorName::two, auto_aim::ArmorPriority::first},
                               {auto_aim::ArmorName::one, auto_aim::ArmorPriority::second},
                               {auto_aim::ArmorName::three, auto_aim::ArmorPriority::second},
                               {auto_aim::ArmorName::four, auto_aim::ArmorPriority::second},
                               {auto_aim::ArmorName::five, auto_aim::ArmorPriority::second},
                               {auto_aim::ArmorName::sentry, auto_aim::ArmorPriority::third},
                               {auto_aim::ArmorName::outpost, auto_aim::ArmorPriority::third},
                               {auto_aim::ArmorName::base, auto_aim::ArmorPriority::third},
                               {auto_aim::ArmorName::not_armor, auto_aim::ArmorPriority::third}};
};

/**
 * @brief 由TOML选择的策略优先级模式，决定不同编号装甲板的攻击顺序
 */
enum PriorityMode { MODE_ONE = 1, MODE_TWO };

} // namespace omniperception

#endif