/**
 * @file yolov8.hpp
 * @brief YOLOv8 OpenVINO后端接口，封装该模型版本的预处理、推理和分类后处理
 */

#ifndef AUTO_AIM__YOLOV8_HPP
#define AUTO_AIM__YOLOV8_HPP

#include <list>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <string>
#include <vector>

#include "app/auto_aim/armor.hpp"
#include "app/auto_aim/classifier.hpp"
#include "app/auto_aim/detector.hpp"
#include "app/auto_aim/yolo.hpp"

namespace auto_aim {

/**
 * @brief 基于OpenVINO的YOLOv8装甲板检测后端，常与数字分类器配合使用
 */
class YOLOV8 : public YOLOBase {
  public:
    /**
     * @brief 从TOML加载YOLOv8模型和运行参数，便于快速切换模型和设备
     * @param config_path TOML配置文件路径，用于读取模型、阈值和运行参数
     * @param debug 是否启用调试图像和检测结果保存，便于离线分析误检原因
     */
    YOLOV8(const std::string& config_path, bool debug);

    /**
     * @brief 对单帧图像执行预处理、推理和后处理，输出可被PnP使用的装甲板对象
     * @param bgr_img 原始BGR图像帧，函数会在内部完成缩放、推理和坐标还原
     * @param frame_count 可选帧编号，用于调试保存文件和定位异常帧
     * @return 检测得到的装甲板列表，已完成后处理和基础过滤
     */
    std::list<Armor> detect(const cv::Mat& bgr_img, int frame_count) override;

    /**
     * @brief 将模型输出解码为装甲板检测结果，并完成置信度过滤和NMS
     * @param scale 预处理letterbox缩放比例，用于把网络输出还原到原图坐标系
     * @param output 模型原始输出张量，函数会按当前后端约定解析它
     * @param bgr_img 原始BGR图像帧，用于坐标还原、数字裁剪和调试绘制
     * @param frame_count 可选帧编号，用于调试保存文件和定位异常帧
     * @return 检测得到的装甲板列表，已完成后处理和基础过滤
     */
    std::list<Armor> postprocess(double scale, cv::Mat& output, const cv::Mat& bgr_img,
                                 int frame_count) override;

  private:
    Classifier classifier_;
    Detector detector_;

    std::string device_, model_path_;
    std::string save_path_, debug_path_;
    bool debug_, use_roi_;

    const int class_num_ = 2;
    const float nms_threshold_ = 0.3;
    const float score_threshold_ = 0.7;
    double min_confidence_, binary_threshold_;

    ov::Core core_;
    ov::CompiledModel compiled_model_;

    cv::Rect roi_;
    cv::Point2f offset_;

    bool check_name(const Armor& armor) const;
    bool check_type(const Armor& armor) const;

    cv::Mat get_pattern(const cv::Mat& bgr_img, const Armor& armor) const;
    ArmorType get_type(const Armor& armor);
    cv::Point2f get_center_norm(const cv::Mat& bgr_img, const cv::Point2f& center) const;

    std::list<Armor> parse(double scale, cv::Mat& output, const cv::Mat& bgr_img, int frame_count);

    void save(const Armor& armor) const;
    void draw_detections(const cv::Mat& img, const std::list<Armor>& armors, int frame_count) const;
    void sort_keypoints(std::vector<cv::Point2f>& keypoints);
};

} // namespace auto_aim

#endif // TOOLS__YOLOV8_HPP