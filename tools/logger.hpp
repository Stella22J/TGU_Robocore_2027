/**
 * @file logger.hpp
 * @brief 轻量级线程安全日志工具接口声明。
 */

#ifndef TGU_ROBOCORE_2027_LOGGER_HPP
#define TGU_ROBOCORE_2027_LOGGER_HPP

#pragma once

#include <fstream>
#include <mutex>
#include <string>
#include <fmt/format.h>

namespace tools {
    /**
     * @brief 日志输出等级。
     */
    enum class LogLevel {
        Debug = 0, ///< 调试日志。
        Info,     ///< 普通信息日志。
        Warn,     ///< 警告日志。
        Error,    ///< 错误日志。
        Off       ///< 关闭日志输出。
    };

    /**
     * @brief Logger 初始化配置。
     */
    struct LoggerConfig {
        // 最低输出日志等级。
        LogLevel level = LogLevel::Debug;
        // 是否输出到控制台。
        bool enable_console = true;
        // 是否输出到文件。
        bool enable_file = false;
        // 日志文件路径。
        std::string file_path = "log.txt";
    };

    /**
     * @brief 单例日志器。
     */
    class Logger {
    public:
        /**
         * @brief 获取 Logger 单例。
         * @return Logger 单例引用。
         */
        static Logger &instance();

        /**
         * @brief 初始化日志器配置。
         * @param config 日志配置。
         */
        void init(const LoggerConfig &config);

        /**
         * @brief 输出一条日志。
         * @param level 日志等级。
         * @param module 模块名。
         * @param msg 日志正文。
         * @param file 调用处源文件路径。
         * @param line 调用处源码行号。
         */
        void log(LogLevel level,
                 const std::string &module,
                 const std::string &msg,
                 const char *file,
                 int line);

    private:
        Logger() = default;

        ~Logger();

        std::string format(LogLevel level,
                           const std::string &module,
                           const std::string &msg,
                           const char *file,
                           int line) const;

        LogLevel level_ = LogLevel::Debug;
        bool console_ = true;
        bool file_ = false;

        std::ofstream ofs_;
        std::mutex mutex_;
    };
} // namespace tools

/**
 * @brief 输出 Info 级别日志。
 * @param module 模块名。
 * @param fmt_str fmt 格式化字符串。
 * @param ... fmt 格式化参数。
 */
#define LOG_INFO(module, fmt_str, ...) \
::tools::Logger::instance().log( \
::tools::LogLevel::Info, module, \
fmt::format(fmt_str __VA_OPT__(,) __VA_ARGS__), \
__FILE__, __LINE__)

#ifdef NDEBUG
/**
 * @brief Release 构建下编译期移除 Debug 级别日志。
 * @param module 模块名。
 * @param fmt_str fmt 格式化字符串。
 * @param ... fmt 格式化参数。
 */
#define LOG_DEBUG(module, fmt_str, ...) ((void)0)
#else
/**
 * @brief 输出 Debug 级别日志。
 * @param module 模块名。
 * @param fmt_str fmt 格式化字符串。
 * @param ... fmt 格式化参数。
 */
#define LOG_DEBUG(module, fmt_str, ...) \
::tools::Logger::instance().log( \
::tools::LogLevel::Debug, module, \
fmt::format(fmt_str __VA_OPT__(,) __VA_ARGS__), \
__FILE__, __LINE__)
#endif

/**
 * @brief 输出 Warn 级别日志。
 * @param module 模块名。
 * @param fmt_str fmt 格式化字符串。
 * @param ... fmt 格式化参数。
 */
#define LOG_WARN(module, fmt_str, ...) \
::tools::Logger::instance().log( \
::tools::LogLevel::Warn, module, \
fmt::format(fmt_str __VA_OPT__(,) __VA_ARGS__), \
__FILE__, __LINE__)

/**
 * @brief 输出 Error 级别日志。
 * @param module 模块名。
 * @param fmt_str fmt 格式化字符串。
 * @param ... fmt 格式化参数。
 */
#define LOG_ERROR(module, fmt_str, ...) \
::tools::Logger::instance().log( \
::tools::LogLevel::Error, module, \
fmt::format(fmt_str __VA_OPT__(,) __VA_ARGS__), \
__FILE__, __LINE__)

#endif //TGU_ROBOCORE_2027_LOGGER_HPP
