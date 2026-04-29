/**
 * @file exiter.hpp
 * @brief 程序退出信号处理工具接口声明。
 * @namespace tools
 */

#ifndef TOOLS__EXITER_HPP
#define TOOLS__EXITER_HPP

namespace tools {
class Exiter {
  public:
    Exiter();

    bool exit() const;
};

} // namespace tools

#endif // TOOLS__EXITER_HPP