/**
 * @file img_tools.cpp
 * @brief OpenCV 图像绘制工具函数实现。
 *
 * 本文件实现图像调试绘制函数，包括绘制单点、点集轮廓和文本。
 * 主要用于视觉算法调试、检测结果可视化、图像标注和运行状态显示。
 *
 * @namespace tools
 */

#include "img_tools.hpp"

namespace tools {
void draw_point(cv::Mat& img, const cv::Point& point, const cv::Scalar& color, int radius) {
    cv::circle(img, point, radius, color, -1);
}

void draw_points(cv::Mat& img, const std::vector<cv::Point>& points, const cv::Scalar& color,
                 int thickness) {
    std::vector<std::vector<cv::Point>> contours = {points};
    cv::drawContours(img, contours, -1, color, thickness);
}

void draw_points(cv::Mat& img, const std::vector<cv::Point2f>& points, const cv::Scalar& color,
                 int thickness) {
    std::vector<cv::Point> int_points(points.begin(), points.end());
    draw_points(img, int_points, color, thickness);
}

void draw_text(cv::Mat& img, const std::string& text, const cv::Point& point,
               const cv::Scalar& color, double font_scale, int thickness) {
    cv::putText(img, text, point, cv::FONT_HERSHEY_SIMPLEX, font_scale, color, thickness);
}

} // namespace tools