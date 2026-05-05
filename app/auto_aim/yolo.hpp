/**
 * @file yolo.hpp
 * @brief YOLO装甲板检测门面，对上层隐藏不同模型版本的预处理和后处理差异
 */

#ifndef AUTO_AIM__YOLO_HPP
#define AUTO_AIM__YOLO_HPP

#include <opencv2/opencv.hpp>

#include "armor.hpp"

namespace auto_aim {
/**
 * @brief YOLO后端运行时多态接口，使异步检测器和普通检测流程可以复用同一抽象
 *
 * 不同YOLO后端的张量排布不同，因此用统一接口隔离差异，避免上层模块随模型版本变化而改动
 */
class YOLOBase {
  public:
    /**
     * @brief 执行完整检测流程，包含预处理、推理、后处理和装甲板对象构造
     * @param img 输入图像，调用方不需要关心具体模型的预处理尺寸
     * @param frame_count 可选帧编号，用于调试保存文件和定位异常帧
     * @return 检测得到的装甲板列表，已完成后处理和基础过滤
     */
    virtual std::list<Armor> detect(const cv::Mat& img, int frame_count) = 0;

    /**
     * @brief 把原始网络输出转换为装甲板列表，便于异步推理复用后处理代码
     * @param scale 预处理letterbox缩放比例，用于把网络输出还原到原图坐标系
     * @param output 网络原始输出矩阵，具体布局由各YOLO后端解析
     * @param bgr_img 原始BGR图像帧，用于坐标还原、数字裁剪和调试绘制
     * @param frame_count 可选帧编号，用于调试保存文件和定位异常帧
     * @return 检测得到的装甲板列表，已完成后处理和基础过滤
     */
    virtual std::list<Armor> postprocess(double scale, cv::Mat& output, const cv::Mat& bgr_img,
                                         int frame_count) = 0;
};

/**
 * @brief 根据配置选择并持有一个YOLO后端，避免业务代码直接依赖具体模型版本
 *
 * 外观类让上层模块不直接依赖具体YOLO版本，切换模型时只需要修改配置
 */
class YOLO {
  public:
    /**
     * @brief 从TOML创建YOLO后端，使模型版本和设备选择可以通过配置切换
     * @param config_path TOML配置文件路径，其中包含`yolo_name`和各模型路径
     * @param debug 是否启用后端相关调试输出，正式运行时通常关闭以保证实时性
     */
    YOLO(const std::string& config_path, bool debug = true);

    /**
     * @brief 使用已选择的后端处理单帧图像，输出统一的装甲板列表
     * @param img 输入图像，调用方不需要关心具体模型的预处理尺寸
     * @param frame_count 可选帧编号，用于调试保存文件和定位异常帧
     * @return 检测得到的装甲板列表，已完成后处理和基础过滤
     */
    std::list<Armor> detect(const cv::Mat& img, int frame_count = -1);

    /**
     * @brief 复用当前后端的后处理逻辑，保证同步和异步推理输出一致
     * @param scale 预处理letterbox缩放比例，用于把网络输出还原到原图坐标系
     * @param output 网络原始输出矩阵，具体布局由各YOLO后端解析
     * @param bgr_img 原始BGR图像帧，用于坐标还原、数字裁剪和调试绘制
     * @param frame_count 可选帧编号，用于调试保存文件和定位异常帧
     * @return 检测得到的装甲板列表，已完成后处理和基础过滤
     */
    std::list<Armor> postprocess(double scale, cv::Mat& output, const cv::Mat& bgr_img,
                                 int frame_count);

  private:
    std::unique_ptr<YOLOBase> yolo_;
};

} // namespace auto_aim

#endif // AUTO_AIM__YOLO_HPP