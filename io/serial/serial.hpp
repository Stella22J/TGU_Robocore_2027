#ifndef IO__SERIAL_HPP
#define IO__SERIAL_HPP

/**
 * @file serial.hpp
 * @brief 声明Boost.Asio串口封装和结构体帧解析工具。
 *
 * 该文件提供同步串口读写和按结构体帧头解析的轻量工具，方便IMU等固定协议设备复用。
 */

#include <boost/asio.hpp>

#include <cstring>
#include <functional>
#include <string>
#include <vector>

namespace io {

/**
 * @brief 固定容量环形缓冲区。
 *
 * 环形缓冲可以在串口字节流错位时逐字节滑动匹配帧头，而不需要频繁移动大块内存。
 *
 * @tparam N 缓冲区容量。
 */
template <size_t N>
class RingBuffer {
  public:
    /**
     * @brief 写入一个字节。
     *
     * 缓冲区满时覆盖最旧数据，使解析器可以在持续字节流中自动恢复同步。
     *
     * @param byte 待写入字节。
     */
    void push(uint8_t byte) {
        buffer_[write_] = byte;
        write_ = (write_ + 1) % N;

        if (size_ < N) {
            size_++;
        } else {
            read_ = (read_ + 1) % N;
        }
    }

    /**
     * @brief 读取相对队首的字节。
     *
     * @param idx 相对队首偏移。
     * @return 对应字节。
     */
    uint8_t at(size_t idx) const { return buffer_[(read_ + idx) % N]; }

    /**
     * @brief 丢弃队首若干字节。
     *
     * @param n 丢弃字节数。
     */
    void pop(size_t n) {
        read_ = (read_ + n) % N;
        size_ -= n;
    }

    /**
     * @brief 获取当前缓存字节数。
     *
     * @return 当前缓存字节数。
     */
    size_t size() const { return size_; }

  private:
    uint8_t buffer_[N]; // 固定数组避免串口接收过程中动态分配
    size_t read_ = 0;  // 队首位置
    size_t write_ = 0; // 下一次写入位置
    size_t size_ = 0;  // 当前有效字节数
};

/**
 * @brief 基于结构体帧头的串口解析器。
 *
 * 解析器假设协议帧头等于结构体默认值的前2字节，适合固定帧头、固定长度的二进制协议。
 *
 * @tparam T 协议结构体类型。
 * @tparam N 内部环形缓冲区容量。
 */
template <typename T, size_t N = 2048>
class StructParser {
  public:
    /**
     * @brief 构造解析器。
     *
     * 从T的默认构造对象提取帧头，避免调用方重复传入协议常量。
     */
    StructParser() {
        T dummy{};
        std::memcpy(head_, &dummy, HEAD_SIZE);
    }

    /**
     * @brief 输入一个字节并尝试解析完整结构体。
     *
     * @param byte 输入字节。
     * @param[out] out 解析成功时输出结构体。
     * @return 成功解析完整帧时返回true。
     */
    bool input(uint8_t byte, T& out) {
        buffer_.push(byte);

        while (buffer_.size() >= sizeof(T)) {
            if (!match_head()) {
                buffer_.pop(1);
                continue;
            }

            if (buffer_.size() < sizeof(T)) {
                return false;
            }

            uint8_t temp[sizeof(T)];
            for (size_t i = 0; i < sizeof(T); i++) {
                temp[i] = buffer_.at(i);
            }

            std::memcpy(&out, temp, sizeof(T));
            buffer_.pop(sizeof(T));
            return true;
        }

        return false;
    }

  private:
    static constexpr size_t HEAD_SIZE = 2;

    bool match_head() {
        for (size_t i = 0; i < HEAD_SIZE; i++) {
            if (buffer_.at(i) != head_[i]) {
                return false;
            }
        }
        return true;
    }

    RingBuffer<N> buffer_;       // 保留最近字节用于错位重同步
    uint8_t head_[HEAD_SIZE];    // 固定帧头提高匹配速度
};

/**
 * @brief Boost.Asio同步串口封装。
 *
 * 该类把串口参数设置、同步读写和结构体回调解析封装在一起，降低各设备驱动重复代码量。
 */
class Serial {
  public:
    /**
     * @brief 构造串口对象。
     */
    Serial();

    /**
     * @brief 析构串口对象。
     *
     * 析构时关闭串口，避免设备文件描述符泄漏。
     */
    ~Serial();

    /**
     * @brief 打开串口。
     *
     * @param device 串口设备路径。
     * @param baudrate 波特率。
     * @return 打开成功返回true。
     */
    bool open(const std::string& device, int baudrate);

    /**
     * @brief 关闭串口。
     */
    void close();

    /**
     * @brief 查询串口是否打开。
     *
     * @return 串口可用时返回true。
     */
    bool is_open() const;

    /**
     * @brief 写入原始字节。
     *
     * @param data 数据指针。
     * @param size 数据长度。
     * @return 实际写入字节数。
     */
    size_t write(const uint8_t* data, size_t size);

    /**
     * @brief 读取指定长度的原始字节。
     *
     * @param data 输出缓冲区。
     * @param size 期望读取字节数。
     * @return 实际读取字节数。
     */
    size_t read(uint8_t* data, size_t size);

    /**
     * @brief 按结构体二进制布局发送数据。
     *
     * @tparam T 数据结构体类型。
     * @param data 待发送数据。
     * @return 实际写入字节数。
     */
    template <typename T>
    size_t send(const T& data) {
        return write(reinterpret_cast<const uint8_t*>(&data), sizeof(T));
    }

    /**
     * @brief 注册结构体接收回调。
     *
     * 回调通过StructParser从字节流中恢复完整结构体，适合固定长度协议帧。
     *
     * @tparam T 协议结构体类型。
     * @param cb 完整帧回调。
     */
    template <typename T>
    void recv(std::function<void(const T&)> cb) {
        auto wrapper = [this, cb](const uint8_t* data, size_t len) {
            for (size_t i = 0; i < len; i++) {
                T pkt;
                if (parser_<T>.input(data[i], pkt)) {
                    cb(pkt);
                }
            }
        };

        callbacks_.push_back(wrapper);
    }

    /**
     * @brief 执行一次非阻塞接收分发。
     *
     * 该函数适合由外部循环驱动，避免Serial内部强行创建线程。
     */
    void spin_once();

  private:
    boost::asio::io_context io_;       // Asio串口对象需要io_context持有底层执行环境
    boost::asio::serial_port serial_;  // 使用Asio统一跨平台串口访问
    bool is_open_;                     // 缓存打开状态，异常路径可快速返回

    uint8_t rx_buf_[512]; // 固定接收缓冲减少spin_once()分配

    std::vector<std::function<void(const uint8_t*, size_t)>> callbacks_; // 多协议回调共享同一串口数据源

    template <typename T>
    static StructParser<T> parser_;

    static constexpr const char* MODULE = "SERIAL";
};

template <typename T>
StructParser<T> Serial::parser_;

} // namespace io

#endif // IO__SERIAL_HPP
