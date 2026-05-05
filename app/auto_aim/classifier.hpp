/**
 * @file classifier.hpp
 * @brief 装甲板数字分类器接口，隔离OpenCV DNN和OpenVINO推理细节
 */

#ifndef AUTO_AIM__CLASSIFIER_HPP
#define AUTO_AIM__CLASSIFIER_HPP

#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <string>

#include "armor.hpp"

namespace auto_aim {
/**
 * @brief 对裁剪后的装甲板数字区域分类，使跟踪和策略能基于目标编号工作
 */
class Classifier {
  public:
    /**
     * @brief 从TOML加载分类模型路径和运行参数，方便不同部署设备切换模型
     * @param config_path TOML配置文件路径，其中需要包含分类模型路径`classify_model`
     */
    explicit Classifier(const std::string& config_path);

    /**
     * @brief 使用OpenCV DNN分类数字，作为轻量或兼容性更好的推理路径
     * @param armor 待分类的装甲板，函数会根据pattern更新其语义编号和置信度
     */
    void classify(Armor& armor);

    /**
     * @brief 使用OpenVINO分类数字，在Intel平台上获得更稳定的推理性能
     * @param armor 待分类的装甲板，函数会根据pattern更新其语义编号和置信度
     */
    void ovclassify(Armor& armor);

  private:
    cv::dnn::Net net_;
    ov::Core core_;
    ov::CompiledModel compiled_model_;
};

} // namespace auto_aim

#endif // AUTO_AIM__CLASSIFIER_HPP