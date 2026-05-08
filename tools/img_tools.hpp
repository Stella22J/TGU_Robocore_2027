/**
 * @file img_tools.hpp
 * @brief OpenCV 图像绘制工具函数接口声明。、
 */

#ifndef TOOLS__IMG_TOOLS_HPP
#define TOOLS__IMG_TOOLS_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace tools {
/**
 * @brief 在图像上绘制一个实心点。
 * @param img 待绘制图像。
 * @param point 点坐标。
 * @param color 绘制颜色。
 * @param radius 点半径。
 */
void draw_point(cv::Mat& img, const cv::Point& point, const cv::Scalar& color = {0, 0, 255},
                int radius = 3);

/**
 * @brief 在图像上按轮廓方式绘制整型点集。
 * @param img 待绘制图像。
 * @param points 点集。
 * @param color 绘制颜色。
 * @param thickness 线宽。
 */
void draw_points(cv::Mat& img, const std::vector<cv::Point>& points,
                 const cv::Scalar& color = {0, 0, 255}, int thickness = 2);

/**
 * @brief 在图像上按轮廓方式绘制浮点点集。
 * @param img 待绘制图像。
 * @param points 点集。
 * @param color 绘制颜色。
 * @param thickness 线宽。
 */
void draw_points(cv::Mat& img, const std::vector<cv::Point2f>& points,
                 const cv::Scalar& color = {0, 0, 255}, int thickness = 2);

/**
 * @brief 在图像上绘制文本。
 * @param img 待绘制图像。
 * @param text 文本内容。
 * @param point 文本基准点。
 * @param color 文本颜色。
 * @param font_scale 字体缩放系数。
 * @param thickness 线宽。
 */
void draw_text(cv::Mat& img, const std::string& text, const cv::Point& point,
               const cv::Scalar& color = {0, 255, 255}, double font_scale = 1.0, int thickness = 2);

} // namespace tools

#endif // TOOLS__IMG_TOOLS_HPP