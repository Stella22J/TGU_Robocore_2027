/**
 * @file detector.hpp
 * @brief 传统灯条装甲板检测接口，封装阈值、几何约束和数字分类流程
 */

#ifndef AUTO_AIM__DETECTOR_HPP
#define AUTO_AIM__DETECTOR_HPP

#include <list>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "armor.hpp"
#include "classifier.hpp"

namespace auto_aim {

/**
 * @brief 通过灯条几何检测装甲板，便于在网络检测不稳定时提供可解释的候选目标
 */
class Detector {
  public:
    /**
     * @brief 从TOML构造几何检测器，使阈值可以按相机和赛场光照调整
     * @param config_path TOML配置文件路径，其中包含几何阈值和数字分类模型路径
     * @param debug 是否启用调试图像和数字区域保存，比赛运行时通常关闭以保证实时性
     */
    Detector(const std::string& config_path, bool debug = true);

    /**
     * @brief 在BGR图像中检测候选装甲板，并输出经过几何和语义筛选的结果
     * @param bgr_img 输入的BGR图像帧，通常来自主相机
     * @param frame_count 可选帧编号，用于调试保存文件和定位异常帧
     * @return 经过几何、数字和类型筛选后的装甲板候选列表
     */
    std::list<Armor> detect(const cv::Mat& bgr_img, int frame_count = -1);

    /**
     * @brief 用传统几何修正神经网络检测框，提升PnP角点精度
     * @param armor 神经网络检测出的装甲板候选，函数会尝试用传统灯条结果修正其角点
     * @param bgr_img 原始BGR图像帧，用于从候选区域重新提取灯条
     * @return 是否成功得到可信的灯条修正结果
     */
    bool detect(Armor& armor, const cv::Mat& bgr_img);

    friend class YOLOV8;

  private:
    Classifier classifier_;

    double threshold_;
    double max_angle_error_;
    double min_lightbar_ratio_, max_lightbar_ratio_;
    double min_lightbar_length_;
    double min_armor_ratio_, max_armor_ratio_;
    double max_side_ratio_;
    double min_confidence_;
    double max_rectangular_error_;

    bool debug_;
    std::string save_path_;

    // 利用PCA回归角点，参考自https://github.com/CSU-FYT-Vision/FYT2024_vision
    void lightbar_points_corrector(Lightbar& lightbar, const cv::Mat& gray_img) const;

    bool check_geometry(const Lightbar& lightbar) const;
    bool check_geometry(const Armor& armor) const;
    bool check_name(const Armor& armor) const;
    bool check_type(const Armor& armor) const;

    Color get_color(const cv::Mat& bgr_img, const std::vector<cv::Point>& contour) const;
    cv::Mat get_pattern(const cv::Mat& bgr_img, const Armor& armor) const;
    ArmorType get_type(const Armor& armor);
    cv::Point2f get_center_norm(const cv::Mat& bgr_img, const cv::Point2f& center) const;

    void save(const Armor& armor) const;
    void show_result(const cv::Mat& binary_img, const cv::Mat& bgr_img,
                     const std::list<Lightbar>& lightbars, const std::list<Armor>& armors,
                     int frame_count) const;
};

} // namespace auto_aim

#endif // AUTO_AIM__DETECTOR_HPP