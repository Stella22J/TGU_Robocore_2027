#ifndef IO__CAMERA_HPP
#define IO__CAMERA_HPP

/**
 * @file camera.hpp
 * @brief 声明相机统一接口和运行时工厂。
 */

#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

namespace io {

/**
 * @brief 相机驱动抽象基类。
 */
class CameraBase {
  public:
    /**
     * @brief 默认虚析构函数。
     */
    virtual ~CameraBase() = default;

    /**
     * @brief 读取一帧图像及其采集时间戳。
     * @param img 输出图像。
     * @param timestamp 图像对应的本机稳态时钟时间戳。
     */
    virtual void read(cv::Mat& img, std::chrono::steady_clock::time_point& timestamp) = 0;
};

/**
 * @brief 相机统一入口。
 */
class Camera {
  public:
    /**
     * @brief 根据TOML配置文件构造相机对象。
     * @param config_path TOML配置文件路径。
     * @throws std::runtime_error 配置缺失、类型错误、解析失败或相机类型不受支持时抛出。
     */
    explicit Camera(const std::string& config_path);

    /**
     * @brief 读取一帧图像及其采集时间戳。
     * @param img 输出图像。
     * @param timestamp 图像对应的本机稳态时钟时间戳。
     */
    void read(cv::Mat& img, std::chrono::steady_clock::time_point& timestamp);

  private:
    // 用基类指针保存真实驱动，方便后续扩展更多相机类型
    std::unique_ptr<CameraBase> camera_;
};

} // namespace io

#endif // IO__CAMERA_HPP
