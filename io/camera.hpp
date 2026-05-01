#ifndef IO__CAMERA_HPP
#define IO__CAMERA_HPP

/**
 * @file camera.hpp
 * @brief 相机统一接口与工厂封装类声明。
 *
 * 本文件定义了相机模块的统一抽象接口CameraBase。
 *
 * CameraBase
 * 用于屏蔽不同相机SDK的差异，使MindVision、HikRobot等具体相机驱动都能够通过统一的read()接口输出图像和时间戳。
 *
 * @namespace io
 */

#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

namespace io {

/**
 * @brief 相机驱动抽象基类
 *
 * 所有具体相机驱动类都应继承该类,为上层视觉算法提供统一图像输入接口，避免依赖相机SDK
 */
class CameraBase {
  public:
    virtual ~CameraBase() = default;

    /**
     * @brief 读取一帧图像及其时间戳。
     *
     * 具体行为由派生类实现。
     *
     * @param img 输出图像
     * @param timestamp 图像对应时间戳
     */
    virtual void read(cv::Mat& img, std::chrono::steady_clock::time_point& timestamp) = 0;
};

/**
 * @brief 相机统一入口类。
 *
 * 根据TOML配置文件创建具体相机驱动对象。
 *
 * 上层模块只需要依赖Camera类，而无需关心底层具体相机类型。
 */
class Camera {
  public:
    /**
     * @brief 根据配置文件构造相机对象
     * @param config_path TOML配置文件路径
     */
    explicit Camera(const std::string& config_path);

    /**
     * @brief 读取一帧图像及其时间戳
     * @param img 输出图像
     * @param timestamp 图像对应时间戳
     */
    void read(cv::Mat& img, std::chrono::steady_clock::time_point& timestamp);

  private:
    /**
     * @brief 具体相机驱动对象
     */
    std::unique_ptr<CameraBase> camera_;
};

} // namespace io

#endif // IO__CAMERA_HPP