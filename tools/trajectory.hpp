/**
 * @file trajectory.hpp
 * @brief 抛体弹道解算接口声明。
 */

#ifndef TOOLS__TRAJECTORY_HPP
#define TOOLS__TRAJECTORY_HPP

namespace tools {
/**
 * @brief 不考虑空气阻力的抛体弹道解算结果。
 */
struct Trajectory {
    // 是否无可行弹道解。
    bool unsolvable;
    // 飞行时间，单位为 s。
    double fly_time;
    //发射俯仰角，抬头为正。
    double pitch;

    /**
     * @brief 解算不考虑空气阻力的弹道。
     * @param v0 子弹初速度大小，单位 m/s。
     * @param d 目标水平距离，单位 m。
     * @param h 目标竖直高度，单位 m。
     */
    Trajectory(const double v0, const double d, const double h);
};

} // namespace tools

#endif // TOOLS__TRAJECTORY_HPP