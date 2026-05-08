/**
 * @file armor.hpp
 * @brief 自瞄装甲板和灯条数据模型，识别、位姿解算和跟踪阶段都依赖这些统一字段
 */

#ifndef AUTO_AIM__ARMOR_HPP
#define AUTO_AIM__ARMOR_HPP

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace auto_aim {
// 视觉识别到的装甲灯颜色，用于过滤己方目标和保留裁判系统约定的特殊颜色
enum Color { red, blue, extinguish, purple };
const std::vector<std::string> COLORS = {"red", "blue", "extinguish", "purple"};

// 装甲板物理尺寸类型，PnP必须选择正确模型才能得到可信三维位置
enum ArmorType { big, small };
const std::vector<std::string> ARMOR_TYPES = {"big", "small"};

// 装甲板语义编号，策略决策和通信协议都用它表达目标身份
enum ArmorName { one, two, three, four, five, sentry, outpost, base, not_armor };
const std::vector<std::string> ARMOR_NAMES = {"one",    "two",     "three", "four",     "five",
                                              "sentry", "outpost", "base",  "not_armor"};

// 目标优先级枚举，数值越小表示越应该优先攻击
enum ArmorPriority { first = 1, second, third, forth, fifth };

// clang-format off
const std::vector<std::tuple<Color, ArmorName, ArmorType>> armor_properties = {
  {blue, sentry, small},     {red, sentry, small},     {extinguish, sentry, small},
  {blue, one, small},        {red, one, small},        {extinguish, one, small},
  {blue, two, small},        {red, two, small},        {extinguish, two, small},
  {blue, three, small},      {red, three, small},      {extinguish, three, small},
  {blue, four, small},       {red, four, small},       {extinguish, four, small},
  {blue, five, small},       {red, five, small},       {extinguish, five, small},
  {blue, outpost, small},    {red, outpost, small},    {extinguish, outpost, small},
  {blue, base, big},         {red, base, big},         {extinguish, base, big},      {purple, base, big},       
  {blue, base, small},       {red, base, small},       {extinguish, base, small},    {purple, base, small},    
  {blue, three, big},        {red, three, big},        {extinguish, three, big}, 
  {blue, four, big},         {red, four, big},         {extinguish, four, big},  
  {blue, five, big},         {red, five, big},         {extinguish, five, big}};
// clang-format on

/**
 * @brief 单根灯条的几何描述，传统视觉需要用它筛掉异常轮廓并组合装甲板
 */
struct Lightbar {
    std::size_t id;
    Color color;
    cv::Point2f center, top, bottom, top2bottom;
    std::vector<cv::Point2f> points;
    double angle, angle_error, length, width, ratio;
    cv::RotatedRect rotated_rect;

    /**
     * @brief 由轮廓拟合矩形构造灯条，同时计算后续几何筛选需要的中心、端点和比例信息
     * @param rotated_rect OpenCV根据轮廓拟合得到的最小外接旋转矩形
     * @param id 灯条在当前帧中的递增编号，用于发现共用灯条和去除重复装甲板
     */
    Lightbar(const cv::RotatedRect& rotated_rect, std::size_t id);
    /**
     * @brief 构造空灯条，便于容器占位和延迟赋值
     */
    Lightbar() {};
};

/**
 * @brief 装甲板检测结果，保存图像位置、语义类别和位姿解算结果以贯通整条自瞄链路
 */
struct Armor {
    Color color;
    Lightbar left, right;    // 保留灯条副本而不是const引用，避免后续修正角点时产生悬空引用
    cv::Point2f center;      // 不是对角线交点，不能作为实际中心！
    cv::Point2f center_norm; // 归一化坐标
    std::vector<cv::Point2f> points;

    double ratio;             // 两灯条的中点连线与长灯条的长度之比
    double side_ratio;        // 长灯条与短灯条的长度之比
    double rectangular_error; // 灯条和中点连线所成夹角与π/2的差值

    ArmorType type;
    ArmorName name;
    ArmorPriority priority;
    int class_id;
    cv::Rect box;
    cv::Mat pattern;
    double confidence;
    bool duplicated;

    Eigen::Vector3d xyz_in_gimbal; // 单位：m
    Eigen::Vector3d xyz_in_world;  // 单位：m
    Eigen::Vector3d ypr_in_gimbal; // 单位：rad
    Eigen::Vector3d ypr_in_world;  // 单位：rad
    Eigen::Vector3d ypd_in_world;  // 球坐标系

    double yaw_raw; // 单位为弧度，后续三角函数和控制命令都使用弧度

    /**
     * @brief 由左右灯条构造装甲板，传统视觉路径需要保留灯条几何关系用于筛选
     * @param left 装甲板左侧灯条，包含传统视觉检测出的几何信息
     * @param right 装甲板右侧灯条，包含传统视觉检测出的几何信息
     */
    Armor(const Lightbar& left, const Lightbar& right);
    /**
     * @brief 由神经网络类别和关键点构造装甲板，用于不带ROI偏移的检测输出
     * @param class_id 神经网络输出的类别编号，用于映射颜色、编号和装甲板类型
     * @param confidence 神经网络输出的置信度，后续筛选和重复目标判断会使用它
     * @param box 原图坐标系下的检测框，用于调试显示和裁剪数字区域
     * @param armor_keypoints 原图坐标系下的四个装甲板角点，顺序需要与PnP模型点保持一致
     */
    Armor(int class_id, float confidence, const cv::Rect& box,
          std::vector<cv::Point2f> armor_keypoints);
    /**
     * @brief 由ROI坐标系下的网络输出构造装甲板，并把点位恢复到原图坐标
     * @param class_id 神经网络输出的类别编号，用于映射颜色、编号和装甲板类型
     * @param confidence 神经网络输出的置信度，后续筛选和重复目标判断会使用它
     * @param box ROI坐标系下的检测框，需要结合offset恢复到原图坐标
     * @param armor_keypoints ROI坐标系下的四个角点，需要结合offset恢复到原图坐标
     * @param offset ROI左上角在原图中的偏移，用于避免裁剪推理后坐标系混用
     */
    Armor(int class_id, float confidence, const cv::Rect& box,
          std::vector<cv::Point2f> armor_keypoints, cv::Point2f offset);
    /**
     * @brief 在颜色和编号分开解码时构造装甲板，适配不同YOLO输出头
     * @param color_id 神经网络输出的颜色编号，用于区分红蓝方和特殊装甲颜色
     * @param num_id 神经网络输出的数字编号，用于映射机器人编号或特殊目标
     * @param confidence 神经网络输出的置信度，后续筛选和重复目标判断会使用它
     * @param box 原图坐标系下的检测框，用于调试显示和裁剪数字区域
     * @param armor_keypoints 原图坐标系下的四个装甲板角点，顺序需要与PnP模型点保持一致
     */
    Armor(int color_id, int num_id, float confidence, const cv::Rect& box,
          std::vector<cv::Point2f> armor_keypoints);
    /**
     * @brief 在ROI坐标系下构造颜色和编号分开解码的装甲板，避免后处理阶段丢失原图偏移
     * @param color_id 神经网络输出的颜色编号，用于区分红蓝方和特殊装甲颜色
     * @param num_id 神经网络输出的数字编号，用于映射机器人编号或特殊目标
     * @param confidence 神经网络输出的置信度，后续筛选和重复目标判断会使用它
     * @param box ROI坐标系下的检测框，需要结合offset恢复到原图坐标
     * @param armor_keypoints ROI坐标系下的四个角点，需要结合offset恢复到原图坐标
     * @param offset ROI左上角在原图中的偏移，用于避免裁剪推理后坐标系混用
     */
    Armor(int color_id, int num_id, float confidence, const cv::Rect& box,
          std::vector<cv::Point2f> armor_keypoints, cv::Point2f offset);
};

} // namespace auto_aim

#endif // AUTO_AIM__ARMOR_HPP