/**
 * @file solver.hpp
 * @brief 装甲板位姿解算接口，上层跟踪器只需要使用统一的米制坐标结果
 */

#ifndef AUTO_AIM__SOLVER_HPP
#define AUTO_AIM__SOLVER_HPP

#include <Eigen/Dense> // 必须在opencv2/core/eigen.hpp上面
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>

#include "armor.hpp"

namespace auto_aim {
/**
 * @brief 根据图像关键点和相机标定解算装甲板位姿，把视觉检测结果转换为可控制的空间量
 */
class Solver {
  public:
    /**
     * @brief 从TOML加载相机内参、畸变和刚体外参，避免把实车标定数据写死在代码里
     * @param config_path TOML配置文件路径，其中包含相机内参、畸变系数和坐标变换矩阵
     */
    explicit Solver(const std::string& config_path);

    /**
     * @brief 返回当前云台到世界坐标系旋转，用于调试和跨模块坐标一致性检查
     * @return 当前云台到世界坐标系的旋转矩阵，用于把装甲板位置投影到统一坐标系
     */
    Eigen::Matrix3d R_gimbal2world() const;

    /**
     * @brief 根据IMU姿态更新世界坐标系变换，使目标位置不随云台姿态变化而漂移
     * @param q IMU输出的四元数，需与配置中的机体系约定保持一致
     */
    void set_R_gimbal2world(const Eigen::Quaterniond& q);

    /**
     * @brief 为单块装甲板计算云台系和世界系位姿，结果会写回Armor对象供跟踪使用
     * @param armor 带有图像角点的装甲板，函数会在原对象中写入米制位姿结果
     */
    void solve(Armor& armor) const;

    /**
     * @brief 把三维装甲板模型重投影到图像上，用于评估当前位姿假设是否可信
     * @param armor 已完成位姿解算的装甲板，用于生成重投影点
     * @param pitch 候选俯仰角，用于评估前哨站等特殊目标的重投影误差
     * @return 根据当前相机模型重投影得到的图像坐标点
     */
    std::vector<cv::Point2f> reproject_armor(const Eigen::Vector3d& xyz_in_world, double yaw,
                                             ArmorType type, ArmorName name) const;

    /**
     * @brief 计算前哨站在给定pitch假设下的重投影误差，用于处理其特殊安装角
     * @param armor 用于局部优化的装甲板副本，避免搜索过程中污染原始观测
     * @param picth 候选俯仰角，保留原拼写是为了兼容已有调用代码
     * @return 像素级重投影误差，数值越小表示候选姿态越可信
     */
    double oupost_reprojection_error(Armor armor, const double& picth);

    /**
     * @brief 将世界坐标点投影到相机图像，用于可视化和调试坐标变换链路
     * @param worldPoints 世界坐标系下的三维点集，用于调试和验证坐标变换
     * @return 经过当前相机模型投影后的像素坐标
     */
    std::vector<cv::Point2f> world2pixel(const std::vector<cv::Point3f>& worldPoints);

  private:
    cv::Mat camera_matrix_;
    cv::Mat distort_coeffs_;
    Eigen::Matrix3d R_gimbal2imubody_;
    Eigen::Matrix3d R_camera2gimbal_;
    Eigen::Vector3d t_camera2gimbal_;
    Eigen::Matrix3d R_gimbal2world_;

    void optimize_yaw(Armor& armor) const;

    double armor_reprojection_error(const Armor& armor, double yaw, const double& inclined) const;
    double SJTU_cost(const std::vector<cv::Point2f>& cv_refs,
                     const std::vector<cv::Point2f>& cv_pts, const double& inclined) const;
};

} // namespace auto_aim

#endif // AUTO_AIM__SOLVER_HPP