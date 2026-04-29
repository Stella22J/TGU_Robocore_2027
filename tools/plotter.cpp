/**
 * @file plotter.cpp
 * @brief UDP 绘图数据发送器实现。
 *
 * 本文件实现 Plotter 类，通过 UDP socket 将 JSON 序列化数据发送到指定主机
 * 和端口。发送过程使用互斥锁保护，适用于多线程场景下的实时调试数据输出。
 * 主要用于将算法状态、曲线数据或中间变量发送给外部绘图工具进行可视化。
 *
 * @namespace tools
 */

#include "plotter.hpp"

#include <arpa/inet.h>  // htons, inet_addr
#include <sys/socket.h> // socket, sendto
#include <unistd.h>     // close

namespace tools {
Plotter::Plotter(std::string host, uint16_t port) {
    socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);

    destination_.sin_family = AF_INET;
    destination_.sin_port = ::htons(port);
    destination_.sin_addr.s_addr = ::inet_addr(host.c_str());
}

Plotter::~Plotter() {
    ::close(socket_);
}

void Plotter::plot(const nlohmann::json& json) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto data = json.dump();
    ::sendto(socket_, data.c_str(), data.length(), 0, reinterpret_cast<sockaddr*>(&destination_),
             sizeof(destination_));
}

} // namespace tools