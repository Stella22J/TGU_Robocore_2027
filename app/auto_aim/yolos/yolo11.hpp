/**
 * @file yolo11.hpp
 * @brief YOLO11 OpenVINO后端接口，封装模型加载、推理和后处理
 */

#ifndef AUTO_AIM__YOLO11_HPP
#define AUTO_AIM__YOLO11_HPP

#include <list>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <string>
#include <vector>

#include "app/auto_aim/armor.hpp"
#include "app/auto_aim/detector.hpp"
#include "app/auto_aim/yolo.hpp"

namespace auto_aim {
/**
 * @brief 基于OpenVINO的YOLO11装甲板检测后端，服务于更高版本模型部署
 *
 * 后端解析逻辑单独放置，是因为不同YOLO版本的输出维度、类别组织和关键点顺序并不相同
 */
class YOLO11 : public YOLOBase {
  public:
    /**
     * @brief 从TOML加载YOLO11模型和运行参数，便于在不同设备上调整推理配置
     * @param config_path TOML配置文件路径，用于读取模型、阈值和运行参数
     * @param debug 是否启用调试图像和检测结果保存，便于离线分析误检原因
     */
    YOLO11(const std::string& config_path, bool debug);

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
    std::string device_, model_path_;
    std::string save_path_, debug_path_;
    bool debug_, use_roi_;

    const int class_num_ = 38;
    const float nms_threshold_ = 0.3;
    const float score_threshold_ = 0.7;
    double min_confidence_, binary_threshold_;

    ov::Core core_;
    ov::CompiledModel compiled_model_;

    cv::Rect roi_;
    cv::Point2f offset_;
    cv::Mat tmp_img_;

    Detector detector_;

    bool check_name(const Armor& armor) const;
    bool check_type(const Armor& armor) const;

    cv::Point2f get_center_norm(const cv::Mat& bgr_img, const cv::Point2f& center) const;

    std::list<Armor> parse(double scale, cv::Mat& output, const cv::Mat& bgr_img, int frame_count);

    void save(const Armor& armor) const;
    void draw_detections(const cv::Mat& img, const std::list<Armor>& armors, int frame_count) const;
    void sort_keypoints(std::vector<cv::Point2f>& keypoints);
};

} // namespace auto_aim

#endif // AUTO_AIM__YOLO11_HPP