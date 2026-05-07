#ifndef IO__SOCKETCAN_HPP
#define IO__SOCKETCAN_HPP

/**
 * @file socketcan.hpp
 * @brief 声明LinuxSocketCAN通信封装。
 *
 * 该文件把socket、epoll和重连逻辑封装在一个类中，让上层协议代码只关心CAN帧收发。
 */

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

namespace io {

// 固定上限可以避免接收线程中动态分配事件数组
static constexpr int MAX_EVENTS = 10;

/**
 * @brief LinuxSocketCAN通信封装类。
 *
 * 该类负责打开CAN原始socket、后台接收CAN帧、发送CAN帧，并在接收异常后自动重连。
 */
class SocketCAN {
  public:
    /**
     * @brief 构造SocketCAN对象。
     *
     * 构造时立即尝试打开接口并启动守护线程，使控制板连接恢复对上层透明。
     *
     * @param interface CAN接口名。
     * @param rx_handler 收到CAN帧后的回调函数。
     */
    SocketCAN(const std::string& interface, std::function<void(const can_frame& frame)> rx_handler)
        : interface_(interface), socket_fd_(-1), epoll_fd_(-1), quit_(false), ok_(false),
          rx_handler_(rx_handler) {
        // 构造时先尝试打开，成功时立即进入后台接收
        try_open();

        // 守护线程负责恢复异常接收线程，避免主流程处理硬件热插拔
        daemon_thread_ = std::thread{[this] {
            while (!quit_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                // 接收线程正常时守护线程只保持低频检查
                if (ok_) {
                    continue;
                }
                // 异常线程必须先回收，再重建socket资源
                if (read_thread_.joinable()) {
                    read_thread_.join();
                }
                close();
                try_open();
            }
        }};
    }

    /**
     * @brief 析构SocketCAN对象。
     *
     * 析构时先通知线程退出再关闭文件描述符，避免接收线程继续访问已关闭的socket。
     */
    ~SocketCAN() {
        quit_ = true;

        if (daemon_thread_.joinable()) {
            daemon_thread_.join();
        }
        if (read_thread_.joinable()) {
            read_thread_.join();
        }

        close();
        LOG_INFO("SOCKETCAN", "SocketCAN destructed.");
    }

    /**
     * @brief 发送一帧CAN数据。
     *
     * @param frame 待发送的CAN帧指针。
     * @throws std::runtime_error socket未打开或写入失败时抛出。
     */
    void write(can_frame* frame) const {
        // 无效socket直接报错，避免静默丢失控制帧
        if (socket_fd_ == -1) {
            throw std::runtime_error("SocketCAN is not opened!");
        }

        if (::write(socket_fd_, frame, sizeof(can_frame)) == -1) {
            throw std::runtime_error("Unable to write!");
        }
    }

  private:
    std::string interface_; // 从配置传入，便于不同平台使用can0或can1
    int socket_fd_;         // 用-1作为无效值，便于重连时判断资源状态
    int epoll_fd_;          // 单独保存epoll句柄，便于异常路径统一关闭
    bool quit_;             // 线程共享退出标志，析构时用于结束后台循环
    bool ok_;               // 守护线程通过该标志判断是否需要重连

    std::thread read_thread_;   // 独立接收线程避免阻塞上层逻辑
    std::thread daemon_thread_; // 守护线程隔离重连逻辑

    can_frame frame_;                 // 复用缓存，避免接收循环频繁分配
    epoll_event events_[MAX_EVENTS];  // 固定数组满足高频CAN收包场景

    std::function<void(const can_frame& frame)> rx_handler_; // 把协议解析交给上层，SocketCAN只负责传输

    /**
     * @brief 打开CAN接口并启动接收线程。
     *
     * 使用epoll等待可读事件，可以避免接收线程长期阻塞在recv上，析构和异常恢复更可控。
     *
     * @throws std::runtime_error socket、ioctl、bind或epoll配置失败时抛出。
     */
    void open() {
        // 创建Linux原生CANsocket
        socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (socket_fd_ < 0) {
            throw std::runtime_error("Error opening socket!");
        }

        // 根据接口名查询内核接口索引
        ifreq ifr;
        std::memset(&ifr, 0, sizeof(ifreq));
        std::strncpy(ifr.ifr_name, interface_.c_str(), IFNAMSIZ - 1);

        if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
            ::close(socket_fd_);
            socket_fd_ = -1;
            throw std::runtime_error("Error getting interface index!");
        }

        // 绑定到指定CAN接口，只接收该总线数据
        sockaddr_can addr;
        std::memset(&addr, 0, sizeof(sockaddr_can));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(socket_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(sockaddr_can)) < 0) {
            ::close(socket_fd_);
            socket_fd_ = -1;
            throw std::runtime_error("Error binding socket to interface!");
        }

        // epoll带超时等待，便于线程收到退出标志
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

        // 接收线程只做读帧和分发，重连交给守护线程
        read_thread_ = std::thread([this]() {
            ok_ = true;

            while (!quit_) {
                std::this_thread::sleep_for(std::chrono::microseconds(10));

                try {
                    read();
                } catch (const std::exception& e) {
                    LOG_WARN("SOCKETCAN", "SocketCAN::read() failed: {}", e.what());
                    ok_ = false;
                    break;
                }
            }
        });

        LOG_INFO("SOCKETCAN", "SocketCAN opened.");
    }

    /**
     * @brief 尝试打开CAN接口。
     *
     * 失败只记录日志，让守护线程可以继续周期性重试。
     */
    void try_open() {
        try {
            open();
        } catch (const std::exception& e) {
            LOG_WARN("SOCKETCAN", "SocketCAN::open() failed: {}", e.what());
        }
    }

    /**
     * @brief 读取CAN帧并触发回调。
     *
     * @throws std::runtime_error epoll或recv失败时抛出。
     */
    void read() {
        // 小超时让线程能及时响应quit_
        int num_events = epoll_wait(epoll_fd_, events_, MAX_EVENTS, 2);
        if (num_events == -1) {
            throw std::runtime_error("Error waiting for events!");
        }

        for (int i = 0; i < num_events; i++) {
            // 使用非阻塞recv避免epoll误唤醒时卡住接收线程
            ssize_t num_bytes = recv(socket_fd_, &frame_, sizeof(can_frame), MSG_DONTWAIT);
            if (num_bytes == -1) {
                throw std::runtime_error("Error reading from SocketCAN!");
            }

            if (num_bytes == sizeof(can_frame)) {
                // 完整CAN帧才交给协议层处理
                rx_handler_(frame_);
            }
        }
    }

    /**
     * @brief 关闭CANsocket和epoll句柄。
     *
     * 多次调用必须安全，因为异常恢复和析构路径都会清理资源。
     */
    void close() {
        if (socket_fd_ == -1) {
            return;
        }

        if (epoll_fd_ != -1) {
            // 先从epoll移除，避免关闭socket后残留监听关系
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
