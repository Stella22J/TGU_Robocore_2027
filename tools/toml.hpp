/**
 * @file toml.hpp
 * @brief 加载toml配置
 *
 * @namespace tools
 */

#ifndef TOOLS__TOML_HPP
#define TOOLS__TOML_HPP

#include <cstdlib>
#include <string>

#include "tomlpp.hpp"
#include "tools/logger.hpp"

namespace tools
{

/**
 * @brief 加载TOML配置文件
 * @param path TOML配置文件路径
 * @return 解析后的TOML表
 */
inline toml::table load(const std::string & path)
{
    try {
        return toml::parse_file(path);
    } catch (const toml::parse_error & e) {
        logger()->error(
          "[TOML] Failed to parse file: {}\n"
          "[TOML] Error: {}\n"
          "[TOML] Source: {}",
          path,
          e.description(),
          e.source().begin);
        exit(1);
    }
}

/**
 * @brief 从TOML表中读取指定类型配置项
 * @tparam T 配置项类型
 * @param toml TOML配置表
 * @param key 配置项名称
 * @return 配置项值
 */
template <typename T>
inline T read(const toml::table & toml, const std::string & key)
{
    auto value = toml[key].value<T>();

    if (value) {
        return *value;
    }

    logger()->error("[TOML] {} not found or type mismatch!", key);
    exit(1);
}

}  // namespace tools

#endif  // TOOLS__TOML_HPP