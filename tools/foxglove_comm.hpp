/**
 * @file foxglove_comm.hpp
 * @brief Foxglove WebSocket 通信封装接口声明。
 */

#ifndef TGU_ROBOCORE_2027_FOXGLOVE_COMM_HPP
#define TGU_ROBOCORE_2027_FOXGLOVE_COMM_HPP

#pragma once

#include <memory>
#include <string>

namespace tools {
/**
 * @brief Foxglove WebSocket 服务端通信封装。
 */
class FoxGloveComm {
  public:
    /**
     * @brief 创建 Foxglove WebSocket 服务端。
     * @param host 绑定主机地址。
     * @param port 绑定端口。
     */
    FoxGloveComm(const std::string& host = "0.0.0.0", uint16_t port = 8765);

    /**
     * @brief 销毁通信封装对象。
     */
    ~FoxGloveComm();

    /** @brief 禁止拷贝构造。 */
    FoxGloveComm(const FoxGloveComm&) = delete;

    /** @brief 禁止拷贝赋值。 */
    FoxGloveComm& operator=(const FoxGloveComm&) = delete;

    /** @brief 支持移动构造。 */
    FoxGloveComm(FoxGloveComm&&) noexcept;

    /** @brief 支持移动赋值。 */
    FoxGloveComm& operator=(FoxGloveComm&&) noexcept;

    /**
     * @brief 查询服务端是否初始化成功。
     * @return 初始化成功返回 true，否则返回 false。
     */
    bool ok() const;

    /**
     * @brief 获取当前绑定主机地址。
     * @return 当前绑定主机地址引用。
     */
    const std::string& host() const;

    /**
     * @brief 获取当前绑定端口。
     * @return 当前绑定端口。
     */
    uint16_t port() const;

    /**
     * @brief 使用新的绑定参数重新创建 server。
     * @param host 新的绑定主机地址。
     * @param port 新的绑定端口。
     * @return 重新创建成功返回 true，否则返回 false。
     */
    bool reset(const std::string& host, uint16_t port);

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace tools

#endif // TGU_ROBOCORE_2027_FOXGLOVE_COMM_HPP
