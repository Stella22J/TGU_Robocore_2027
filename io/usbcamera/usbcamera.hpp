#ifndef IO__USBCAMERA_HPP
#define IO__USBCAMERA_HPP

/**
 * @file usbcamera.hpp
 * @brief 声明基于OpenCVVideoCapture的USB相机驱动。
 *
 * 该类用于接入普通V4LUSB摄像头，并提供与工业相机类似的带时间戳读取接口。
 */

#include <chrono>
#include <iostream>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

#include "tools/thread_safe_queue.hpp"

namespace io {

/**
 * @brief USB相机驱动类。
 *
 * 使用后台采集线程持续取图，并用守护线程在读帧失败后尝试重新打开设备。
 */
class USBCamera {
  public:
    /**
     * @brief 构造USB相机对象。
     *
     * @param open_name 设备名，例如"video0"。
     * @param config_path TOML配置文件路径。
     */
    USBCamera(const std::string& open_name, const std::string& config_path);

    /**
     * @brief 析构USB相机对象。
     *
     * 析构时结束后台线程并释放VideoCapture，避免设备节点被占用。
     */
    ~USBCamera();

    /**
     * @brief 同步读取一帧图像。
     *
     * 保留该接口用于简单调试场景，正式链路优先使用带时间戳的read()。
     *
     * @return 读取到的图像，失败时返回空Mat。
     */
    cv::Mat read();

    /**
     * @brief 读取一帧图像及其采集时间戳。
     *
     * @param[out] img 输出图像。
     * @param[out] timestamp 图像对应的本机稳态时钟时间戳。
     */
    void read(cv::Mat& img, std::chrono::steady_clock::time_point& timestamp);

    std::string device_name; // 公开设备名，便于外部日志区分左右相机

  private:
    struct CameraData {
        cv::Mat img;                                      // cv::Mat引用计数可减少队列传递成本
        std::chrono::steady_clock::time_point timestamp;  // 使用本机时间戳支持多传感器对齐
    };

    std::mutex cap_mutex_; // VideoCapture不是线程安全对象，需要集中保护
    cv::VideoCapture cap_; // OpenCV负责V4L设备访问，降低普通USB相机接入成本
    cv::Mat img_;          // 同步read()复用缓存，减少调试接口分配

    std::string open_name_; // 保存原始设备名，重连时无需依赖外部状态

    double usb_exposure_;    // OpenCV属性接口使用double，避免隐式转换
    double usb_frame_rate_;  // OpenCV属性接口使用double，便于保留配置精度
    int sharpness_;          // 复用设备参数区分左右相机，兼容当前部署方式
    int open_count_;         // 限制重试次数，避免设备不存在时无限刷日志

    double image_width_;  // 分辨率放在配置里，现场调试无需重新编译
    double image_height_; // 分辨率放在配置里，现场调试无需重新编译
    double usb_gamma_;    // Gamma放在配置里，便于不同光照环境快速调整
    double usb_gain_;     // 增益放在配置里，便于不同光照环境快速调整

    bool quit_; // 控制守护线程和采集线程退出
    bool ok_;   // 守护线程通过该状态判断是否需要重启

    std::thread capture_thread_; // 隔离阻塞式VideoCapture读帧
    std::thread daemon_thread_;  // 独立处理异常恢复

    tools::ThreadSafeQueue<CameraData> queue_; // 小容量队列优先保留新帧，避免图像积压

    /**
     * @brief 尝试打开USB相机。
     *
     * 打开失败只记录日志并增加计数，守护线程会决定是否继续重试。
     */
    void try_open();

    /**
     * @brief 打开并配置USB相机。
     *
     * 配置和启动采集线程集中在一个函数中，便于异常后完整恢复设备状态。
     */
    void open();

    /**
     * @brief 关闭USB相机。
     *
     * 多次调用需要安全，因为析构和异常恢复路径都会触发清理。
     */
    void close();
};

} // namespace io

#endif // IO__USBCAMERA_HPP
