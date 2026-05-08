/**
 * @file recorder.hpp
 * @brief 图像帧与姿态数据记录器接口声明。
 */

#ifndef TOOLS__RECORDER_HPP
#define TOOLS__RECORDER_HPP

#include <Eigen/Geometry>
#include <chrono>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <thread>

#include "tools/thread_safe_queue.hpp"
namespace tools {
/**
 * @brief 图像帧与姿态数据记录器。
 */
class Recorder {
  public:
    /**
     * @brief 创建记录器。
     * @param fps 输出视频帧率。
     */
    Recorder(double fps = 30);

    /**
     * @brief 停止保存线程并释放文件资源。
     */
    ~Recorder();

    /**
     * @brief 记录一帧图像和对应姿态。
     * @param img 图像帧。
     * @param q 该帧对应的姿态四元数。
     * @param timestamp 该帧时间戳。
     */
    void record(const cv::Mat& img, const Eigen::Quaterniond& q,
                const std::chrono::steady_clock::time_point& timestamp);

  private:
    struct FrameData {
        cv::Mat img;
        Eigen::Quaterniond q;
        std::chrono::steady_clock::time_point timestamp;
    };
    bool init_;
    std::atomic<bool> stop_thread_;
    double fps_;
    std::string text_path_;
    std::string video_path_;
    std::ofstream text_writer_;
    cv::VideoWriter video_writer_;
    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point last_time_;
    tools::ThreadSafeQueue<FrameData> queue_;
    std::thread saving_thread_; // 负责保存帧数据的线程
    void init(const cv::Mat& img);
    void save_to_file();
};

} // namespace tools

#endif // TOOLS__RECORDER_HPP