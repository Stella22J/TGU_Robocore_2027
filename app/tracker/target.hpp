/**
 * @file target.hpp
 * @brief EKF目标状态模型接口，为瞄准预测提供连续、可外推的目标状态
 */

#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <chrono>
#include <optional>
#include <queue>
#include <string>
#include <vector>

#include "../auto_aim/armor.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace auto_aim {

/**
 * @brief 单个敌方机器人目标的EKF状态，描述整车中心、速度、朝向和装甲板几何参数
 *
 * 跟踪器估计整车中心和旋转状态，而不是只跟踪单块装甲板，是为了在装甲板切换时仍保持连续预测
 */
class Target {
  public:
    ArmorName name;
    ArmorType armor_type;
    ArmorPriority priority;
    bool jumped;
    int last_id; // 仅用于调试当前匹配到的装甲板序号

    /**
     * @brief 构造空目标，便于容器保存和状态机延迟初始化
     */
    Target() = default;
    Target(const Armor& armor, std::chrono::steady_clock::time_point t, double radius,
           int armor_num, Eigen::VectorXd P0_dig);
    /**
     * @brief 构造用于规划器测试的目标，方便脱离真实视觉输入验证MPC逻辑
     * @param x 初始目标中心x坐标，单位为米
     * @param vyaw 初始角速度，决定小陀螺等旋转目标的预测趋势
     * @param radius 装甲板到车辆旋转中心的半径，用于从装甲板位置反推车体中心
     * @param h 装甲板相对车辆中心的高度，用于处理高低装甲板切换
     */
    Target(double x, double vyaw, double radius, double h);

    /**
     * @brief 将目标状态预测到绝对时间戳，补偿检测和控制链路延迟
     * @param t 未来预测时间戳，用于得到该时刻的目标状态
     */
    void predict(std::chrono::steady_clock::time_point t);
    /**
     * @brief 按给定时间间隔外推目标状态，用于子弹飞行时间迭代
     * @param dt 预测时间间隔，单位为秒
     */
    void predict(double dt);
    /**
     * @brief 用新的装甲板观测更新EKF，使预测状态持续贴合真实目标
     * @param armor 已完成三维位姿解算的装甲板观测，用于更新扩展卡尔曼滤波器
     */
    void update(const Armor& armor);

    /**
     * @brief 返回当前EKF状态向量，供瞄准器读取速度、角速度和车体几何参数
     * @return 状态向量`x vx y vy z vz a w r l h`，分别表示位置、速度、朝向、角速度和几何参数
     */
    Eigen::VectorXd ekf_x() const;
    /**
     * @brief 提供只读EKF对象，便于调试协方差和滤波状态而不破坏内部估计
     * @return 内部扩展卡尔曼滤波器实例，主要用于调试和高级状态读取
     */
    const tools::ExtendedKalmanFilter& ekf() const;
    /**
     * @brief 根据整车状态预测所有装甲板位置，瞄准器据此选择当前可击打装甲板
     * @return 按`x y z yaw`组织的装甲板位姿列表，用于选择当前最适合击打的装甲板
     */
    std::vector<Eigen::Vector4d> armor_xyza_list() const;

    /**
     * @brief 检查目标估计是否已经不可用，防止长时间丢失后继续使用过期预测
     * @return 滤波器是否发散，用于在观测异常时触发重新初始化
     */
    bool diverged() const;

    /**
     * @brief 检查目标是否已经经过足够更新，避免刚初始化时立即开火
     * @return 目标状态是否已经收敛，未收敛时不宜激进开火
     */
    bool convergened();

    bool isinit = false;

    /**
     * @brief 检查自定义初始化是否完成，用于测试或外部强制目标状态场景
     * @return 目标是否已经初始化，防止上层读取无效状态
     */
    bool checkinit();

  private:
    int armor_num_;
    int switch_count_;
    int update_count_;

    bool is_switch_, is_converged_;

    tools::ExtendedKalmanFilter ekf_;
    std::chrono::steady_clock::time_point t_;

    void update_ypda(const Armor& armor, int id); // 观测量顺序为yaw、pitch、distance、armor_yaw

    Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd& x, int id) const;
    Eigen::MatrixXd h_jacobian(const Eigen::VectorXd& x, int id) const;
};

} // namespace auto_aim

#endif // AUTO_AIM__TARGET_HPP