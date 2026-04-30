#ifndef IO__MINDVISION_HPP
#define IO__MINDVISION_HPP

/**
 * @file mindvision.hpp
 * @brief MindVision工业相机驱动类声明。
 *
 * 本文件声明了MindVision相机封装类，用于通过MindVision Camera
 * SDK控制工业相机，并向上层提供统一的CameraBase图像读取接口。
 *
 * @namespace io
 */

#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "CameraApi.h"
#include "io/camera.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io {

/**
 * @brief MindVision工业相机驱动类
 *
 * 该类继承自CameraBase，对MindVision Camera SDK进行封装。
 *
 * 类内部包含两个后台线程：
 * - capture_thread_：图像采集线程，负责从相机SDK获取图像
 * - daemon_thread_：守护线程，负责检测采集状态，并在异常时尝试重启相机
 * 外部模块通过read()接口阻塞式读取最新图像和对应时间戳。
 */
class MindVision : public CameraBase {
  public:
    /**
     * @brief 构造MindVision相机对象
     * @param exposure_ms 曝光时间
     * @param gamma 伽马参数，内部设置SDK时会乘以100。
     * @param vid_pid USB设备VID/PID字符串
     */
    MindVision(double exposure_ms, double gamma, const std::string& vid_pid);

    /**
     * @brief 析构MindVision相机对象。
     *
     * 析构时会通知后台线程退出，等待守护线程和采集线程结束，然后关闭 MindVision 相机。
     */
    ~MindVision() override;

    /**
     * @brief 读取一帧图像及其时间戳。
     *
     * 从线程安全队列中取出一帧图像数据。
     * 如果当前队列为空，该函数会阻塞，直到采集线程写入新图像。
     *
     * @param img输出图像
     * @param timestamp图像获取时间戳
     */
    void read(cv::Mat& img, std::chrono::steady_clock::time_point& timestamp) override;

  private:
    /**
     * @brief 相机图像数据包。
     *
     * 用于在线程安全队列中传递图像和对应时间戳。
     */
    struct CameraData {
        cv::Mat img;                                     // OpenCV 图像数据
        std::chrono::steady_clock::time_point timestamp; // 图像获取时间戳
    };

    double exposure_ms_; // 曝光时间
    double gamma_;       // 伽马参数

    CameraHandle handle_; // MindVision SDK相机句柄
    int height_;          // 图像高度
    int width_;           // 图像宽度

    bool quit_; // 后台线程退出标志
    bool ok_;   // 相机采集状态标志

    std::thread capture_thread_; // 图像采集线程
    std::thread daemon_thread_;  // 相机守护线程

    tools::ThreadSafeQueue<CameraData> queue_; // 图像数据线程安全队列

    int vid_; // USB Vendor ID，用于reset_usb()
    int pid_; // USB Product ID，用于reset_usb()

    /**
     * @brief 打开并配置MindVision相机。
     *
     * 该函数会初始化SDK、枚举相机、初始化相机句柄、获取相机能力、设置曝光/伽马/输出格式/采集模式，并启动图像采集线程。
     *
     */
    void open();

    /**
     * @brief 安全尝试打开相机。
     */
    void try_open();

    /**
     * @brief 关闭MindVision相机。
     *
     * 如果当前相机句柄有效，则调用CameraUnInit()释放相机资源。
     */
    void close();

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
     * 根据vid_和pid_查找USB设备，并调用libusb_reset_device()对设备进行重置。
     */
    void reset_usb() const;
};

} // namespace io

#endif // IO__MINDVISION_HPP