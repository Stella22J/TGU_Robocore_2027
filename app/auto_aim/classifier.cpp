/**
 * @file classifier.cpp
 * @brief 装甲板数字分类器实现，给只负责定位的检测模型补充语义编号
 */

#include "classifier.hpp"

#include "tools/toml.hpp"

namespace auto_aim {
Classifier::Classifier(const std::string& config_path) {
    auto config = tools::load(config_path);
    auto model = tools::read<std::string>(config, "classify_model");
    net_ = cv::dnn::readNetFromONNX(model);
    auto ovmodel = core_.read_model(model);
    compiled_model_ = core_.compile_model(
        ovmodel, "AUTO", ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
}

void Classifier::classify(Armor& armor) {
    if (armor.pattern.empty()) {
        armor.name = ArmorName::not_armor;
        return;
    }

    cv::Mat gray;
    cv::cvtColor(armor.pattern, gray, cv::COLOR_BGR2GRAY);

    auto input = cv::Mat(32, 32, CV_8UC1, cv::Scalar(0));
    auto x_scale = static_cast<double>(32) / gray.cols;
    auto y_scale = static_cast<double>(32) / gray.rows;
    auto scale = std::min(x_scale, y_scale);
    auto h = static_cast<int>(gray.rows * scale);
    auto w = static_cast<int>(gray.cols * scale);

    if (h == 0 || w == 0) {
        armor.name = ArmorName::not_armor;
        return;
    }
    auto roi = cv::Rect(0, 0, w, h);
    cv::resize(gray, input(roi), {w, h});

    auto blob = cv::dnn::blobFromImage(input, 1.0 / 255.0, cv::Size(), cv::Scalar());

    net_.setInput(blob);
    cv::Mat outputs = net_.forward();

    // 将网络输出归一化为概率，便于按最大置信度选择类别
    float max = *std::max_element(outputs.begin<float>(), outputs.end<float>());
    cv::exp(outputs - max, outputs);
    float sum = cv::sum(outputs)[0];
    outputs /= sum;

    double confidence;
    cv::Point label_point;
    cv::minMaxLoc(outputs.reshape(1, 1), nullptr, &confidence, nullptr, &label_point);
    int label_id = label_point.x;

    armor.confidence = confidence;
    armor.name = static_cast<ArmorName>(label_id);
}

void Classifier::ovclassify(Armor& armor) {
    if (armor.pattern.empty()) {
        armor.name = ArmorName::not_armor;
        return;
    }

    cv::Mat gray;
    cv::cvtColor(armor.pattern, gray, cv::COLOR_BGR2GRAY);

    // 归一化到32x32以匹配分类模型输入
    auto input = cv::Mat(32, 32, CV_8UC1, cv::Scalar(0));
    auto x_scale = static_cast<double>(32) / gray.cols;
    auto y_scale = static_cast<double>(32) / gray.rows;
    auto scale = std::min(x_scale, y_scale);
    auto h = static_cast<int>(gray.rows * scale);
    auto w = static_cast<int>(gray.cols * scale);

    if (h == 0 || w == 0) {
        armor.name = ArmorName::not_armor;
        return;
    }

    auto roi = cv::Rect(0, 0, w, h);
    cv::resize(gray, input(roi), {w, h});
    // 归一化输入图像，保持与模型训练阶段的数值范围一致
    input.convertTo(input, CV_32F, 1.0 / 255.0);

    ov::Tensor input_tensor(ov::element::f32, {1, 1, 32, 32}, input.data);

    ov::InferRequest infer_request = compiled_model_.create_infer_request();
    infer_request.set_input_tensor(input_tensor);
    infer_request.infer();

    auto output_tensor = infer_request.get_output_tensor();
    auto output_shape = output_tensor.get_shape();
    cv::Mat outputs(1, 9, CV_32F, output_tensor.data());

    // 计算softmax概率，便于后续按类别置信度筛选
    float max = *std::max_element(outputs.begin<float>(), outputs.end<float>());
    cv::exp(outputs - max, outputs);
    float sum = cv::sum(outputs)[0];
    outputs /= sum;

    double confidence;
    cv::Point label_point;
    cv::minMaxLoc(outputs.reshape(1, 1), nullptr, &confidence, nullptr, &label_point);
    int label_id = label_point.x;

    armor.confidence = confidence;
    armor.name = static_cast<ArmorName>(label_id);
}

} // namespace auto_aim