#ifndef IO__MINDVISION_HPP
#define IO__MINDVISION_HPP

/**
 * @file mindvision.hpp
 * @brief 声明MindVision工业相机驱动。
 */

#include <chrono>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

#include "CameraApi.h"
#include "io/camera.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io {

/**
 * @brief MindVision工业相机驱动类。
 */
class MindVision : public CameraBase {
  public:
    /**
     * @brief 构造MindVision相机对象。
     * @param exposure_ms 曝光时间，单位ms。
     * @param gamma 伽马参数，设置SDK时会按设备要求放大100倍。
     * @param vid_pid USB设备VID/PID字符串，用于异常恢复时重置USB。
     */
    MindVision(double exposure_ms, double gamma, const std::string& vid_pid);

    /**
     * @brief 析构MindVision相机对象。
     */
    ~MindVision() override;

    /**
     * @brief 读取一帧图像及其采集时间戳。
     * @param img 输出图像。
     * @param timestamp 图像对应的本机稳态时钟时间戳。
     */
    void read(cv::Mat& img, std::chrono::steady_clock::time_point& timestamp) override;

  private:
    struct CameraData {
        cv::Mat img;                                      // cv::Mat引用计数可减少队列传递成本
        std::chrono::steady_clock::time_point timestamp;  // 使用本机时间戳支持多传感器对齐
    };

    double exposure_ms_; // 保留配置单位，设置SDK时再转换为us
    double gamma_;       // 保留配置值，设置SDK时再转换为设备尺度

    CameraHandle handle_; // SDK句柄集中在驱动类中管理生命周期

    int height_; // 使用SDK能力信息创建输出图像
    int width_;  // 使用SDK能力信息创建输出图像

    bool quit_; // 控制守护线程和采集线程退出
    bool ok_;   // 守护线程通过该状态判断是否需要重启

    std::thread capture_thread_; // 隔离阻塞式SDK取图调用
    std::thread daemon_thread_;  // 独立处理异常恢复

    tools::ThreadSafeQueue<CameraData> queue_; // 小容量队列优先保留新帧，避免图像积压

    int vid_; // reset_usb()需要VID定位设备
    int pid_; // reset_usb()需要PID定位设备

    /**
     * @brief 打开并配置MindVision相机。
     */
    void open();

    /**
     * @brief 尝试打开相机。
     */
    void try_open();

    /**
     * @brief 关闭MindVision相机。
     */
    void close();

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

#endif // IO__MINDVISION_HPP
