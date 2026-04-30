#ifndef IO__HIKROBOT_HPP
#define IO__HIKROBOT_HPP

/**
 * @file hikrobot.hpp
 * @brief HikRobot工业相机驱动类声明。
 *
 * 本文件声明了HikRobot相机封装类，用于通过HikRobot MVS SDK控制USB工业相机，并向上层提供统一的
 * CameraBase 读取接口。
 *
 * @namespace io
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
 * @brief HikRobot USB工业相机驱动类。
 *
 * 该类继承自CameraBase，对HikRobot MVS SDK进行封装。
 * 类内部包含两类后台线程：
 * - daemon_thread_：守护线程，负责启动采集、监控采集状态，并在异常时重启相机
 * - capture_thread_：采集线程，负责从相机 SDK 获取图像并写入队列
 * 外部模块通过 read() 接口阻塞式读取最新图像和对应时间戳。
 */
class HikRobot : public CameraBase {
  public:
    /**
     * @brief 构造 HikRobot 相机对象。
     * @param exposure_ms 曝光时间，单位：毫秒。
     * @param gain 相机增益。
     * @param vid_pid USB设备VID/PID字符串
     */
    HikRobot(double exposure_ms, double gain, const std::string& vid_pid);

    /**
     * @brief 析构 HikRobot 相机对象。
     *
     * 析构时会通知守护线程退出，并等待守护线程结束。
     */
    ~HikRobot() override;

    /**
     * @brief 读取一帧图像及其时间戳。
     *
     * 从线程安全队列中取出一帧图像数据。
     * 如果当前队列为空，该函数会阻塞，直到采集线程写入新图像。
     *
     * @param img 输出图像
     * @param timestamp 图像获取时间戳
     */
    void read(cv::Mat& img, std::chrono::steady_clock::time_point& timestamp) override;

  private:
    /**
     * @brief 相机图像数据包。
     *
     * 用于在线程安全队列中传递图像和对应时间戳。
     */
    struct CameraData {
        cv::Mat img;
        std::chrono::steady_clock::time_point timestamp; // 图像获取时间戳
    };

    double exposure_us_; // 曝光时间
    double gain_;        // 相机增益

    std::thread daemon_thread_;     // 相机守护线程。
    std::atomic<bool> daemon_quit_; // 守护线程退出标志。

    void* handle_;                   // HikRobot MVS SDK相机句柄。
    std::thread capture_thread_;     // 图像采集线程。
    std::atomic<bool> capturing_;    // 当前是否处于采集状态。
    std::atomic<bool> capture_quit_; // 采集线程退出标志。

    tools::ThreadSafeQueue<CameraData> queue_; // 图像数据线程安全队列。

    int vid_; // USB Vendor ID，用于reset_usb()。
    int pid_; // USB Product ID，用于reset_usb()。

    /**
     * @brief 启动相机采集。
     *
     * 该函数会枚举HikRobot USB相机，创建SDK句柄，打开相机，设置相机参数，启动取流，并创建采集线程。
     */
    void capture_start();

    /**
     * @brief 停止相机采集并释放SDK资源。
     *
     * 该函数会通知采集线程退出，等待线程结束，然后依次调用：
     * - MV_CC_StopGrabbing()
     * - MV_CC_CloseDevice()
     * - MV_CC_DestroyHandle()
     */
    void capture_stop();

    /**
     * @brief 设置相机浮点类型参数。
     * @param name HikRobot SDK参数名。
     * @param value 参数值。
     */
    void set_float_value(const std::string& name, double value);

    /**
     * @brief 设置相机枚举类型参数。
     * @param name HikRobot SDK参数名。
     * @param value 参数枚举值。
     */
    void set_enum_value(const std::string& name, unsigned int value);

    /**
     * @brief 解析USB VID/PID字符串。
     *
     * 输入格式应为十六进制字符串：
     * @code
     * "VID:PID"
     * @endcode
     *
     * @param vid_pid USB VID/PID字符串。
     */
    void set_vid_pid(const std::string& vid_pid);

    /**
     * @brief 重置USB设备。
     *
     * 根据vid_和pid_查USB 设备，并调用libusb_reset_device()对设备进行重置。
     */
    void reset_usb() const;
};

} // namespace io

#endif // IO__HIKROBOT_HPP