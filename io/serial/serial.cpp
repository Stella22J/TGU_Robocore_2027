/**
 * @file serial.cpp
 * @brief 实现Boost.Asio同步串口封装。
 *
 * 该文件集中处理串口打开、读写异常和回调分发，使具体设备驱动只关注协议解析。
 */

#include "serial.hpp"

#include "tools/logger.hpp"

namespace io {

Serial::Serial() : serial_(io_), is_open_(false) {}

Serial::~Serial() { close(); }

bool Serial::open(const std::string& device, int baudrate) {
    try {
        // 重新打开前先关闭旧端口，避免同一对象重复占用设备
        if (serial_.is_open()) {
            serial_.close();
        }

        // 打开物理串口设备
        serial_.open(device);

        // 串口参数与常见8N1协议保持一致
        serial_.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
        serial_.set_option(boost::asio::serial_port_base::character_size(8));
        serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serial_.set_option(
            boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

        is_open_ = true;
        LOG_INFO(MODULE, "{} open success", device);
        return true;
    } catch (const std::exception& e) {
        LOG_ERROR(MODULE, "{} open failed: {}", device, e.what());
        is_open_ = false;
        return false;
    }
}

void Serial::close() {
    if (serial_.is_open()) {
        serial_.close();
    }
    is_open_ = false;
}

bool Serial::is_open() const { return is_open_; }

size_t Serial::write(const uint8_t* data, size_t size) {
    if (!is_open_) {
        return 0;
    }

    try {
        // 同步写入确保调用返回时数据已交给系统串口缓冲
        return boost::asio::write(serial_, boost::asio::buffer(data, size));
    } catch (const std::exception& e) {
        // 异常后标记关闭，让上层可以触发重连或退出
        LOG_ERROR(MODULE, "write failed: {}", e.what());
        is_open_ = false;
        return 0;
    } catch (...) {
        // 异常后标记关闭，让上层可以触发重连或退出
        LOG_ERROR(MODULE, "write failed: unknown exception");
        is_open_ = false;
        return 0;
    }
}

size_t Serial::read(uint8_t* data, size_t size) {
    if (!is_open_) {
        return 0;
    }

    try {
        // 同步读取固定长度数据，适合定长协议帧
        return boost::asio::read(serial_, boost::asio::buffer(data, size));
    } catch (const std::exception& e) {
        LOG_ERROR(MODULE, "read failed: {}", e.what());
        is_open_ = false;
        return 0;
    }
}

void Serial::spin_once() {
    if (!is_open_) {
        return;
    }

    try {
        // 非阻塞式轮询只读取当前可用数据并交给回调解析
        size_t len = serial_.read_some(boost::asio::buffer(rx_buf_));

        // 同一批串口数据分发给所有协议解析器
        for (auto& cb : callbacks_) {
            cb(rx_buf_, len);
        }
    } catch (const std::exception& e) {
        // 读异常后不在这里重连，避免Serial类私自决定设备恢复策略
        LOG_ERROR(MODULE, "spin once failed: {}", e.what());
        is_open_ = false;
    } catch (...) {
        // 读异常后不在这里重连，避免Serial类私自决定设备恢复策略
        LOG_ERROR(MODULE, "spin once failed: unknown exception");
        is_open_ = false;
    }
}

} // namespace io
