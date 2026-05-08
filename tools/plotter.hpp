/**
 * @file plotter.hpp
 * @brief UDP 绘图数据发送器接口声明。
 */

#ifndef TOOLS__PLOTTER_HPP
#define TOOLS__PLOTTER_HPP

#include <netinet/in.h> // sockaddr_in

#include <mutex>
#include <nlohmann/json.hpp>
#include <string>

namespace tools {
/**
 * @brief 通过 UDP 发送 JSON 绘图数据。
 */
class Plotter {
  public:
    /**
     * @brief 创建 UDP 绘图数据发送器。
     * @param host 目标主机地址。
     * @param port 目标端口。
     */
    Plotter(std::string host = "127.0.0.1", uint16_t port = 9870);

    /**
     * @brief 关闭 UDP socket。
     */
    ~Plotter();

    /**
     * @brief 发送一帧 JSON 绘图数据。
     * @param json 待发送的 JSON 数据。
     */
    void plot(const nlohmann::json& json);

  private:
    int socket_;
    sockaddr_in destination_;
    std::mutex mutex_;
};

} // namespace tools

#endif // TOOLS__PLOTTER_HPP