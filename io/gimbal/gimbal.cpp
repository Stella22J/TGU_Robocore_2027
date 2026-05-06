/**
 * @file gimbal.cpp
 * @brief 实现云台串口通信、姿态缓存和控制帧发送
 * @details
 * 云台数据来自连续串口字节流,底层Serial负责根据帧头恢复结构体帧
 * 本文件负责CRC16校验、状态更新、姿态插值和控制帧发送
 * 配置文件使用TOML,是为了和项目其它模块统一配置格式并减少YAML运行时依赖
 */

#include "gimbal.hpp"

#include <cstdlib>
#include <exception>

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/toml.hpp"

namespace io {

Gimbal::Gimbal(const std::string& config_path) {
    auto toml = tools::load(config_path);

    com_port_ = tools::read<std::string>(toml, "com_port");

    // 没有历史默认baudrate来源时,配置文件显式给出更安全
    baudrate_ = tools::read<int>(toml, "baudrate");

    // recv先注册,可以避免串口打开后首帧到达时无人处理
    serial_.recv<GimbalToVision>([this](const GimbalToVision& data) { handle_frame(data); });

    if (!serial_.open(com_port_, baudrate_)) {
        LOG_ERROR("Gimbal", "Failed to open serial:{} at {}", com_port_, baudrate_);
        std::exit(1);
    }

    thread_ = std::thread(&Gimbal::read_thread, this);

    // 等到第一帧姿态后再返回,避免调用方启动后立刻拿到无效姿态
    queue_.pop();
    LOG_INFO("Gimbal", "First q received.");
}

Gimbal::~Gimbal() {
    quit_ = true;

    if (thread_.joinable())
        thread_.join();

    serial_.close();
}

GimbalMode Gimbal::mode() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return mode_;
}

GimbalState Gimbal::state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

std::string Gimbal::str(GimbalMode mode) const {
    switch (mode) {
    case GimbalMode::IDLE:
        return "IDLE";
    case GimbalMode::AUTO_AIM:
        return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF:
        return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF:
        return "BIG_BUFF";
    default:
        return "INVALID";
    }
}

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t) {
    while (true) {
        auto [q_a, t_a] = queue_.pop();
        auto [q_b, t_b] = queue_.front();

        auto t_ab = tools::delta_time(t_a, t_b);
        auto t_ac = tools::delta_time(t_a, t);

        // 用相邻两帧姿态插值,可以补偿图像时间戳和串口接收时间之间的偏差
        auto k = t_ac / t_ab;
        Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();

        // 查询时间早于当前队首时只能返回最近插值结果,避免无限等待旧数据
        if (t < t_a)
            return q_c;

        // 目标时间不在当前两帧之间就继续推进队列
        if (!(t_a < t && t <= t_b))
            continue;

        return q_c;
    }
}

void Gimbal::send(io::VisionToGimbal data) {
    tx_data_.mode = data.mode;
    tx_data_.yaw = data.yaw;
    tx_data_.yaw_vel = data.yaw_vel;
    tx_data_.yaw_acc = data.yaw_acc;
    tx_data_.pitch = data.pitch;
    tx_data_.pitch_vel = data.pitch_vel;
    tx_data_.pitch_acc = data.pitch_acc;

    // CRC必须在所有载荷字段写入后计算,否则下位机会拒收该帧
    tx_data_.crc16 = tools::get_crc16(reinterpret_cast<uint8_t*>(&tx_data_),
                                      sizeof(tx_data_) - sizeof(tx_data_.crc16));

    if (serial_.write(reinterpret_cast<uint8_t*>(&tx_data_), sizeof(tx_data_)) !=
        sizeof(tx_data_)) {
        LOG_WARN("Gimbal", "Failed to write serial");
    }
}

void Gimbal::send(bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch,
                  float pitch_vel, float pitch_acc) {
    // 将业务层的2个布尔量压缩成协议mode,避免上层直接依赖裸数字
    tx_data_.mode = control ? (fire ? 2 : 1) : 0;

    tx_data_.yaw = yaw;
    tx_data_.yaw_vel = yaw_vel;
    tx_data_.yaw_acc = yaw_acc;
    tx_data_.pitch = pitch;
    tx_data_.pitch_vel = pitch_vel;
    tx_data_.pitch_acc = pitch_acc;

    // CRC必须覆盖帧头和全部载荷,但不能把crc16字段本身算进去
    tx_data_.crc16 = tools::get_crc16(reinterpret_cast<uint8_t*>(&tx_data_),
                                      sizeof(tx_data_) - sizeof(tx_data_.crc16));

    if (serial_.write(reinterpret_cast<uint8_t*>(&tx_data_), sizeof(tx_data_)) !=
        sizeof(tx_data_)) {
        LOG_WARN("Gimbal", "Failed to write serial");
    }
}

void Gimbal::handle_frame(const GimbalToVision& data) {
    if (!tools::check_crc16(reinterpret_cast<const uint8_t*>(&data), sizeof(GimbalToVision))) {
        LOG_DEBUG("Gimbal", "CRC16 check failed.");
        return;
    }

    auto t = std::chrono::steady_clock::now();

    // 下位机和协议均使用wxyz顺序,可直接构造Eigen四元数
    Eigen::Quaterniond q(data.q[0], data.q[1], data.q[2], data.q[3]);
    queue_.push({q, t});

    std::lock_guard<std::mutex> lock(mutex_);

    state_.yaw = data.yaw;
    state_.yaw_vel = data.yaw_vel;
    state_.pitch = data.pitch;
    state_.pitch_vel = data.pitch_vel;
    state_.bullet_speed = data.bullet_speed;
    state_.bullet_count = data.bullet_count;

    // 将协议裸数字转换成强类型枚举,避免业务层传播下位机编码细节
    switch (data.mode) {
    case 0:
        mode_ = GimbalMode::IDLE;
        break;
    case 1:
        mode_ = GimbalMode::AUTO_AIM;
        break;
    case 2:
        mode_ = GimbalMode::SMALL_BUFF;
        break;
    case 3:
        mode_ = GimbalMode::BIG_BUFF;
        break;
    default:
        mode_ = GimbalMode::IDLE;
        LOG_WARN("Gimbal", "Invalid mode:{}", data.mode);
        break;
    }
}

void Gimbal::read_thread() {
    LOG_INFO("Gimbal", "read_thread started.");

    int error_count = 0;

    while (!quit_) {
        if (!serial_.is_open()) {
            error_count++;

            if (error_count > 5000) {
                error_count = 0;
                LOG_WARN("Gimbal", "Too many errors, attempting to reconnect...");
                reconnect();
            }

            continue;
        }

        serial_.spin_once();

        if (serial_.is_open()) {
            error_count = 0;
        } else {
            error_count++;
        }
    }

    LOG_INFO("Gimbal", "read_thread stopped.");
}

void Gimbal::reconnect() {
    int max_retry_count = 10;

    for (int i = 0; i < max_retry_count && !quit_; ++i) {
        LOG_WARN("Gimbal", "Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);

        serial_.close();

        try {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        } catch (...) {
            // 等待失败不影响下一次open,这里吞掉异常是为了保证重连流程继续
        }

        if (serial_.open(com_port_, baudrate_)) {
            // 清空旧姿态,避免重连后用断线前数据参与插值
            queue_.clear();

            LOG_INFO("Gimbal", "Reconnected serial successfully.");
            break;
        }

        LOG_WARN("Gimbal", "Reconnect failed");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

} // namespace io