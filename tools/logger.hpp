/**
 * @file logger.hpp
 * @brief 全局日志器接口声明。
 * @namespace tools
 */

#ifndef TOOLS__LOGGER_HPP
#define TOOLS__LOGGER_HPP

#include <spdlog/spdlog.h>

namespace tools {
std::shared_ptr<spdlog::logger> logger();

} // namespace tools

#endif // TOOLS__LOGGER_HPP