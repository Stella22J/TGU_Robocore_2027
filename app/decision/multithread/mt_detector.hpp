/**
 * @file mt_detector.hpp
 * @brief 异步OpenVINO检测器接口，用队列封装OpenVINO异步推理请求
 */

#ifndef AUTO_AIM__MT_DETECTOR_HPP
#define AUTO_AIM__MT_DETECTOR_HPP

#include <chrono>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <tuple>

#include "app/auto_aim/yolos/yolov5.hpp"
#include "tools/logger.hpp"
#include "tools/thread_safe_queue.hpp"

namespace auto_aim {
namespace multithread {

/**
 * @brief 封装YOLO的OpenVINO异步推理，让图像采集、推理和控制可以并行执行
 *
 * 异步推理用于隐藏模型延迟，同时保留原始采集时间戳，避免跟踪器把推理耗时误认为目标运动
 */
class MultiThreadDetector {
  public:
    /**
     * @brief 从TOML创建异步检测器，保持模型路径、设备和预处理参数可配置
     * @param config_path TOML配置文件路径，用于读取模型、阈值和运行参数
     * @param debug 是否启用检测器调试输出，用于保存中间图像和定位异步推理问题
     */
    MultiThreadDetector(const std::string& config_path, bool debug = false);

    /**
     * @brief 提交单帧图像进行异步推理，同时保存时间戳用于后续延迟补偿
     * @param img 待提交异步推理的原始图像，函数内部会复制或缓存必要数据
     * @param t 图像采集时间戳，用于后续跟踪和预测的时间对齐
     */
    void push(cv::Mat img, std::chrono::steady_clock::time_point t);

    /**
     * @brief 等待最早提交的推理完成并返回检测结果，保持输出顺序与输入帧一致
     * @return 检测得到的装甲板列表以及对应的采集时间戳
     */
    std::tuple<std::list<Armor>, std::chrono::steady_clock::time_point> pop(); // 暂时不支持yolov8

    /**
     * @brief 返回原始图像和检测结果，便于调试链路同时查看输入和识别结果
     * @return 原始图像、检测结果和采集时间戳，便于调试异步流程
     */
    std::tuple<cv::Mat, std::list<Armor>, std::chrono::steady_clock::time_point> debug_pop();

  private:
    ov::Core core_;
    ov::CompiledModel compiled_model_;
    std::string device_;
    YOLO yolo_;

    tools::ThreadSafeQueue<
        std::tuple<cv::Mat, std::chrono::steady_clock::time_point, ov::InferRequest>>
        queue_{16, [] { LOG_DEBUG("MultiThreadDetector", "queue is full!"); }};
};

} // namespace multithread

} // namespace auto_aim

#endif // AUTO_AIM__MT_DETECTOR_HPP