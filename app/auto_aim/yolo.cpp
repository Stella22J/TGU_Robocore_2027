/**
 * @file yolo.cpp
 * @brief YOLO装甲板检测门面实现，统一选择YOLOv5、YOLOv8或YOLO11后端
 */

#include "yolo.hpp"

#include "tools/toml.hpp"

#include "yolos/yolo11.hpp"
#include "yolos/yolov5.hpp"
#include "yolos/yolov8.hpp"

namespace auto_aim {
YOLO::YOLO(const std::string& config_path, bool debug) {
    auto config = tools::load(config_path);
    auto yolo_name = tools::read<std::string>(config, "yolo_name");

    if (yolo_name == "yolov8") {
        yolo_ = std::make_unique<YOLOV8>(config_path, debug);
    }

    else if (yolo_name == "yolo11") {
        yolo_ = std::make_unique<YOLO11>(config_path, debug);
    }

    else if (yolo_name == "yolov5") {
        yolo_ = std::make_unique<YOLOV5>(config_path, debug);
    }

    else {
        throw std::runtime_error("Unknown yolo name: " + yolo_name + "!");
    }
}

std::list<Armor> YOLO::detect(const cv::Mat& img, int frame_count) {
    return yolo_->detect(img, frame_count);
}

std::list<Armor> YOLO::postprocess(double scale, cv::Mat& output, const cv::Mat& bgr_img,
                                   int frame_count) {
    return yolo_->postprocess(scale, output, bgr_img, frame_count);
}

} // namespace auto_aim