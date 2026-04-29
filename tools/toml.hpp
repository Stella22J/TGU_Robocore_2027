#ifndef TOOLS__TOML_HPP
#define TOOLS__TOML_HPP

#include <string>

#include "tomlpp.hpp"
#include "tools/logger.hpp"

namespace tools
{
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
          e.source().begin
        );
        exit(1);
    }
}

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