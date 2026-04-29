/**
 * @file exiter.cpp
 * @brief 程序退出信号处理工具实现。
 *
 * 本文件实现 Exiter 类，用于捕获 SIGINT 信号并设置全局退出标志。
 *
 * @namespace tools
 */

#include "exiter.hpp"

#include <csignal>
#include <stdexcept>

namespace tools {
bool exit_ = false;
bool exiter_inited_ = false;

Exiter::Exiter() {
    if (exiter_inited_)
        throw std::runtime_error("Multiple Exiter instances!");
    std::signal(SIGINT, [](int) { exit_ = true; });
    exiter_inited_ = true;
}

bool Exiter::exit() const {
    return exit_;
}

} // namespace tools