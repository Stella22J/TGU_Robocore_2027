/**
 * @file pid.hpp
 * @brief PID 控制器接口声明。
 */

#ifndef TOOLS__PID_HPP
#define TOOLS__PID_HPP

namespace tools {
/**
 * @brief 离散 PID 控制器。
 */
class PID {
  public:
    /**
     * @brief 构造 PID 控制器。
     * @param dt 控制周期，单位为 s。
     * @param kp P 项系数。
     * @param ki I 项系数。
     * @param kd D 项系数。
     * @param max_out PID 最大输出值。
     * @param max_iout I 项最大输出值。
     * @param angular 是否按角度误差处理。
     */
    PID(float dt, float kp, float ki, float kd, float max_out, float max_iout,
        bool angular = false);

    float pout = 0.0f; // P项输出, 用于调试
    float iout = 0.0f; // I项输出, 用于调试
    float dout = 0.0f; // D项输出, 用于调试

    /**
     * @brief 计算 PID 输出值。
     * @param set 目标值。
     * @param fdb 反馈值。
     * @return 限幅后的 PID 输出。
     */
    float calc(float set, float fdb);

  private:
    const float dt_;
    const float kp_, ki_, kd_;
    const float max_out_, max_iout_;
    const bool angular_;

    float last_fdb_ = 0.0f; // 上次反馈值
};

} // namespace tools

#endif // TOOLS__PID_HPP