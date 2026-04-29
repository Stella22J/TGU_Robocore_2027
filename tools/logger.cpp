/**
 * @file logger.cpp
 * @brief 全局日志器实现。
 *
 * 本文件基于 spdlog 创建并维护一个全局 logger 实例，支持控制台彩色输出和文件日志输出。
 *
 * @namespace tools
 */

#include "logger.hpp"

#include <fmt/chrono.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <chrono>
#include <string>

namespace tools {
std::shared_ptr<spdlog::logger> logger_ = nullptr;

void set_logger() {
    auto file_name = fmt::format("logs/{:%Y-%m-%d_%H-%M-%S}.log", std::chrono::system_clock::now());
    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(file_name, true);
    file_sink->set_level(spdlog::level::debug);

    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::debug);

    logger_ =
        std::make_shared<spdlog::logger>("", spdlog::sinks_init_list{file_sink, console_sink});
    logger_->set_level(spdlog::level::debug);
    logger_->flush_on(spdlog::level::info);
}

std::shared_ptr<spdlog::logger> logger() {
    if (!logger_)
        set_logger();
    return logger_;
}

} // namespace tools
