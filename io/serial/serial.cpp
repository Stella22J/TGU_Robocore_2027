/**
 * @file serial.cpp
 * @brief 实现Boost.Asio同步串口封装。
 */

#include "serial.hpp"

#include "tools/logger.hpp"

namespace io {

Serial::Serial() : serial_(io_), is_open_(false) {}

Serial::~Serial() { close(); }

bool Serial::open(const std::string& device, int baudrate) {
    try {
        if (serial_.is_open()) {
            serial_.close();
        }

        serial_.open(device);

        serial_.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
        serial_.set_option(boost::asio::serial_port_base::character_size(8));
        serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serial_.set_option(
            boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none)
        );
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
        return boost::asio::write(serial_, boost::asio::buffer(data, size));
    } catch (const std::exception& e) {
        LOG_ERROR(MODULE, "write failed: {}", e.what());
        is_open_ = false;
        return 0;
    } catch (...) {
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
        size_t len = serial_.read_some(boost::asio::buffer(rx_buf_));

        for (auto& cb : callbacks_) {
            cb(rx_buf_, len);
        }
    } catch (const std::exception& e) {
        LOG_ERROR(MODULE, "spin once failed: {}", e.what());
        is_open_ = false;
    } catch (...) {
        LOG_ERROR(MODULE, "spin once failed: unknown exception");
        is_open_ = false;
    }
}

} // namespace io
