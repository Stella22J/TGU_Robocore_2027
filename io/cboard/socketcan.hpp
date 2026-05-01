/**
 * @file socketcan.hpp
 * @brief 封装LinuxSocketCAN收发逻辑
 * @namespace io
 */

#ifndef IO__SOCKETCAN_HPP
#define IO__SOCKETCAN_HPP

#include <linux/can.h>
#include <net/if.h>
#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <functional>
#include <stdexcept>
#include <string>
#include <thread>

#include "tools/logger.hpp"

using namespace std::chrono_literals;

// epoll单次最多处理的事件数量
constexpr int MAX_EVENTS = 10;

namespace io {

/**
 * @brief LinuxSocketCAN通信封装类
 *
 * 负责打开CAN接口、后台接收CAN帧、发送CAN帧和异常重连
 */
class SocketCAN {
  public:
    /**
     * @brief 构造SocketCAN对象
     * @param interface CAN接口名
     * @param rx_handler 接收CAN帧后的回调函数
     */
    SocketCAN(const std::string& interface, std::function<void(const can_frame& frame)> rx_handler)
        : interface_(interface), socket_fd_(-1), epoll_fd_(-1), quit_(false), ok_(false),
          rx_handler_(rx_handler) {
        try_open();

        // 守护线程,用于检测接收线程状态并尝试重连
        daemon_thread_ = std::thread{[this] {
            while (!quit_) {
                std::this_thread::sleep_for(100ms);

                if (ok_)
                    continue;

                if (read_thread_.joinable())
                    read_thread_.join();

                close();
                try_open();
            }
        }};
    }

    /**
     * @brief 析构SocketCAN对象
     */
    ~SocketCAN() {
        quit_ = true;

        if (daemon_thread_.joinable())
            daemon_thread_.join();
        if (read_thread_.joinable())
            read_thread_.join();

        close();

        tools::logger()->info("SocketCAN destructed.");
    }

    /**
     * @brief 发送一帧CAN数据
     * @param frame 待发送的CAN帧指针
     */
    void write(can_frame* frame) const {
        if (socket_fd_ == -1) {
            throw std::runtime_error("SocketCAN is not opened!");
        }

        if (::write(socket_fd_, frame, sizeof(can_frame)) == -1) {
            throw std::runtime_error("Unable to write!");
        }
    }

  private:
    // CAN接口名
    std::string interface_;

    // CAN原始socket文件描述符
    int socket_fd_;

    // epoll文件描述符
    int epoll_fd_;

    // 线程退出标志
    bool quit_;

    // CAN接收状态标志
    bool ok_;

    // CAN接收线程
    std::thread read_thread_;

    // CAN重连守护线程
    std::thread daemon_thread_;

    // 接收缓存帧
    can_frame frame_;

    // epoll事件缓存
    epoll_event events_[MAX_EVENTS];

    // 收到CAN帧后的回调函数
    std::function<void(const can_frame& frame)> rx_handler_;

    /**
     * @brief 打开CAN接口并启动接收线程
     *
     * 流程:
     * 1.创建PF_CAN/SOCK_RAWsocket
     * 2.通过ioctl获取CAN接口索引
     * 3.将socket绑定到指定CAN接口
     * 4.创建epoll并监听socket可读事件
     * 5.启动接收线程
     */
    void open() {
        socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (socket_fd_ < 0) {
            throw std::runtime_error("Error opening socket!");
        }

        ifreq ifr;
        std::memset(&ifr, 0, sizeof(ifreq));
        std::strncpy(ifr.ifr_name, interface_.c_str(), IFNAMSIZ - 1);

        if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
            ::close(socket_fd_);
            socket_fd_ = -1;
            throw std::runtime_error("Error getting interface index!");
        }

        sockaddr_can addr;
        std::memset(&addr, 0, sizeof(sockaddr_can));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(socket_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(sockaddr_can)) < 0) {
            ::close(socket_fd_);
            socket_fd_ = -1;
            throw std::runtime_error("Error binding socket to interface!");
        }

        epoll_fd_ = epoll_create1(0);
        if (epoll_fd_ == -1) {
            ::close(socket_fd_);
            socket_fd_ = -1;
            throw std::runtime_error("Error creating epoll file descriptor!");
        }

        epoll_event ev;
        std::memset(&ev, 0, sizeof(epoll_event));
        ev.events = EPOLLIN;
        ev.data.fd = socket_fd_;

        if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, ev.data.fd, &ev) == -1) {
            ::close(epoll_fd_);
            ::close(socket_fd_);
            epoll_fd_ = -1;
            socket_fd_ = -1;
            throw std::runtime_error("Error adding socket to epoll file descriptor!");
        }

        // 接收线程
        read_thread_ = std::thread([this]() {
            ok_ = true;

            while (!quit_) {
                std::this_thread::sleep_for(10us);

                try {
                    read();
                } catch (const std::exception& e) {
                    tools::logger()->warn("SocketCAN::read() failed: {}", e.what());
                    ok_ = false;
                    break;
                }
            }
        });

        tools::logger()->info("SocketCAN opened.");
    }

    /**
     * @brief 尝试打开CAN接口
     */
    void try_open() {
        try {
            open();
        } catch (const std::exception& e) {
            tools::logger()->warn("SocketCAN::open() failed: {}", e.what());
        }
    }

    /**
     * @brief 读取CAN帧并触发回调
     */
    void read() {
        int num_events = epoll_wait(epoll_fd_, events_, MAX_EVENTS, 2);
        if (num_events == -1) {
            throw std::runtime_error("Error waiting for events!");
        }

        for (int i = 0; i < num_events; i++) {
            ssize_t num_bytes = recv(socket_fd_, &frame_, sizeof(can_frame), MSG_DONTWAIT);
            if (num_bytes == -1) {
                throw std::runtime_error("Error reading from SocketCAN!");
            }

            if (num_bytes == sizeof(can_frame)) {
                rx_handler_(frame_);
            }
        }
    }

    /**
     * @brief 关闭CANsocket和epoll句柄
     */
    void close() {
        if (socket_fd_ == -1)
            return;

        if (epoll_fd_ != -1) {
            epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, socket_fd_, nullptr);
            ::close(epoll_fd_);
            epoll_fd_ = -1;
        }

        ::close(socket_fd_);
        socket_fd_ = -1;
    }
};

} // namespace io

#endif // IO__SOCKETCAN_HPP