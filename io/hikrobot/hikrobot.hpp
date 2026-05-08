#ifndef IO__HIKROBOT_HPP
#define IO__HIKROBOT_HPP

/**
 * @file hikrobot.hpp
 * @brief 声明HikRobot工业相机驱动。
 */

#include <atomic>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

#include "MvCameraControl.h"
#include "io/camera.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io {

/**
 * @brief HikRobotUSB工业相机驱动类。
 */
class HikRobot : public CameraBase {
  public:
    /**
     * @brief 构造HikRobot相机对象。
     * @param exposure_ms 曝光时间，单位ms。
     * @param gain 相机增益。
     * @param vid_pid USB设备VID/PID字符串，用于异常恢复时重置USB。
     */
    HikRobot(double exposure_ms, double gain, const std::string& vid_pid);

    /**
     * @brief 析构HikRobot相机对象。
     */
    ~HikRobot() override;

    /**
     * @brief 读取一帧图像及其采集时间戳。
     * @param img 输出图像。
     * @param timestamp 图像对应的本机稳态时钟时间戳。
     */
    void read(cv::Mat& img, std::chrono::steady_clock::time_point& timestamp) override;

  private:
    struct CameraData {
        cv::Mat img;                                      // cv::Mat引用计数可安全跨队列传递图像数据
        std::chrono::steady_clock::time_point timestamp;  // 使用本机时间戳支持多传感器对齐
    };

    double exposure_us_; // SDK曝光单位为us，构造时从ms转换后缓存
    double gain_;        // 增益由配置控制，便于现场调参

    std::thread daemon_thread_;        // 守护线程独立处理掉线恢复
    std::atomic<bool> daemon_quit_;    // 原子标志避免析构和守护线程数据竞争

    void* handle_;                    // SDK使用void*句柄，驱动层集中管理生命周期
    std::thread capture_thread_;      // 采集线程隔离阻塞式取图接口
    std::atomic<bool> capturing_;     // 守护线程通过该状态判断是否重启
    std::atomic<bool> capture_quit_;  // 原子标志用于安全停止采集线程

    tools::ThreadSafeQueue<CameraData> queue_; // 容量较小以保留最新图像，避免上层处理滞后积压

    int vid_; // reset_usb()需要VID定位设备
    int pid_; // reset_usb()需要PID定位设备

    /**
     * @brief 启动相机采集。
     */
    void capture_start();

    /**
     * @brief 停止相机采集并释放SDK资源。
     */
    void capture_stop();

    /**
     * @brief 设置相机浮点参数。
     * @param name HikRobotSDK参数名。
     * @param value 参数值。
     */
    void set_float_value(const std::string& name, double value);

    /**
     * @brief 设置相机枚举参数。
     * @param name HikRobotSDK参数名。
     * @param value 参数枚举值。
     */
    void set_enum_value(const std::string& name, unsigned int value);

    /**
     * @brief 解析USBVID/PID字符串。
     * @param vid_pid USB设备VID/PID字符串，格式为"VID:PID"。
     */
    void set_vid_pid(const std::string& vid_pid);

    /**
     * @brief 重置USB设备。
     */
    void reset_usb() const;
};

} // namespace io

#endif // IO__HIKROBOT_HPP
