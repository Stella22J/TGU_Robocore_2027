#ifndef IO__CAMERA_HPP
#define IO__CAMERA_HPP

/**
 * @file camera.hpp
 * @brief 声明相机统一接口和运行时工厂。
 *
 * CameraBase屏蔽不同相机SDK差异，使上层算法只依赖统一的图像和时间戳输出。
 * Camera根据TOML配置创建具体驱动，便于在不改业务代码的情况下切换相机。
 */

#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

namespace io {

/**
 * @brief 相机驱动抽象基类。
 *
 * 具体驱动继承该类后，上层可以通过多态统一读取图像，避免把SDK类型暴露到算法模块。
 */
class CameraBase {
  public:
    /**
     * @brief 默认虚析构函数。
     *
     * 派生类通常由std::unique_ptr<CameraBase>持有，因此需要虚析构保证资源按真实类型释放。
     */
    virtual ~CameraBase() = default;

    /**
     * @brief 读取一帧图像及其采集时间戳。
     *
     * 派生类可以从硬件、后台队列或模拟源取图，但必须保证调用者拿到同一帧对应的时间戳。
     *
     * @param[out] img 输出图像。
     * @param[out] timestamp 图像对应的本机稳态时钟时间戳。
     */
    virtual void read(cv::Mat& img, std::chrono::steady_clock::time_point& timestamp) = 0;
};

/**
 * @brief 相机统一入口。
 *
 * 该类把配置解析和具体驱动创建集中在一处，使调用方不需要关心当前使用MindVision还是HikRobot。
 */
class Camera {
  public:
    /**
     * @brief 根据TOML配置文件构造相机对象。
     *
     * @param config_path TOML配置文件路径。
     * @throws std::runtime_error 配置缺失、类型错误、解析失败或相机类型不受支持时抛出。
     */
    explicit Camera(const std::string& config_path);

    /**
     * @brief 读取一帧图像及其采集时间戳。
     *
     * 该函数只转发到真实驱动，保证上层代码不依赖具体相机实现。
     *
     * @param[out] img 输出图像。
     * @param[out] timestamp 图像对应的本机稳态时钟时间戳。
     */
    void read(cv::Mat& img, std::chrono::steady_clock::time_point& timestamp);

  private:
    // 用基类指针保存真实驱动，方便后续扩展更多相机类型
    std::unique_ptr<CameraBase> camera_;
};

} // namespace io

#endif // IO__CAMERA_HPP
