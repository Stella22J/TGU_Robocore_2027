/**
 * @file toml.hpp
 * @brief TOML配置读取工具
 */

#ifndef TOOLS__TOML_HPP
#define TOOLS__TOML_HPP

#include <cstdlib>
#include <optional>
#include <string>
#include <vector>

#include "tomlpp.hpp"
#include "tools/logger.hpp"

namespace tools
{

/**
 * @brief 加载TOML配置文件
 * @param path TOML配置文件路径
 * @return 解析后的TOML根表
 */
inline toml::table load(const std::string & path)
{
  try {
    return toml::parse_file(path);
  } catch (const toml::parse_error & e) {
    LOG_ERROR("TOML", "Failed to parse file: {}\n"
      "Error: {}\n"
      "Source: {}:{}", path, e.description(), e.source().begin.line, e.source().begin.column);
    exit(1);
  }
}

/**
 * @brief 从TOML表中读取必填配置项
 * @tparam T 配置项目标类型
 * @param table TOML配置表，可以是根表，也可以是子表
 * @param key 配置项名称
 * @return 转换后的配置值
 */
template <typename T>
inline T read(const toml::table & table, const std::string & key)
{
  auto value = table[key].value<T>();

  if (value) {
    return *value;
  }

  LOG_ERROR("TOML", "{} not found or type mismatch!", key);
  exit(1);
}

/**
 * @brief 从TOML表中读取可选配置项
 * @tparam T 配置项目标类型
 * @param table TOML配置表，可以是根表，也可以是子表
 * @param key 配置项名称
 * @return 存在且类型正确时返回配置值，否则返回std::nullopt
 */
template <typename T>
inline std::optional<T> read_optional(const toml::table & table, const std::string & key)
{
  return table[key].value<T>();
}

/**
 * @brief 从TOML表中读取必填子表
 * @param table TOML配置表，可以是根表，也可以是上一级子表
 * @param key 子表名称
 * @return 子表引用
 */
inline const toml::table & read_table(const toml::table & table, const std::string & key)
{
  const auto * child = table[key].as_table();
  if (child != nullptr) {
    return *child;
  }

  LOG_ERROR("TOML", "{} not found or type mismatch!", key);
  exit(1);
}

/**
 * @brief 从TOML表中读取必填数组配置项
 * @tparam T 数组元素目标类型
 * @param table TOML配置表，可以是根表，也可以是子表
 * @param key 数组配置项名称
 * @return 转换后的数组内容
 */
template <typename T>
inline std::vector<T> read_array(const toml::table & table, const std::string & key)
{
  const auto * array = table[key].as_array();
  if (array == nullptr) {
    LOG_ERROR("TOML", "{} not found or type mismatch!", key);
    exit(1);
  }

  std::vector<T> values;
  values.reserve(array->size());
  for (const auto & item : *array) {
    auto value = item.value<T>();
    if (!value) {
      LOG_ERROR("TOML", "{} contains type mismatch!", key);
      exit(1);
    }
    values.emplace_back(*value);
  }

  return values;
}

}  // namespace tools

#endif  // TOOLS__TOML_HPP