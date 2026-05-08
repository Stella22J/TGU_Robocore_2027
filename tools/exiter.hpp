/**
 * @file exiter.hpp
 * @brief 程序退出信号处理工具接口声明。
 */

#ifndef TOOLS__EXITER_HPP
#define TOOLS__EXITER_HPP

namespace tools {
/**
 * @brief SIGINT 退出状态检测器。
 */
class Exiter {
  public:
    /**
     * @brief 注册 SIGINT 信号处理函数。
     */
    Exiter();

    /**
     * @brief 查询是否收到退出信号。
     * @return 收到 SIGINT 时返回 true，否则返回 false。
     */
    bool exit() const;
};

} // namespace tools

#endif // TOOLS__EXITER_HPP