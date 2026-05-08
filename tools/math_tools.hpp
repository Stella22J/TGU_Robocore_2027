/**
 * @file math_tools.hpp
 * @brief 常用数学与几何工具函数接口声明。
 */

#ifndef TOOLS__MATH_TOOLS_HPP
#define TOOLS__MATH_TOOLS_HPP

#include <Eigen/Geometry>
#include <chrono>

namespace tools {
/**
 * @brief 将弧度值限制在 (-pi, pi] 区间。
 * @param angle 输入角度，单位为 rad。
 * @return 归一化后的角度，单位为 rad。
 */
double limit_rad(double angle);

/**
 * @brief 四元数转欧拉角。
 *  x = 0, y = 1, z = 2
 * e.g. 先绕z轴旋转，再绕y轴旋转，最后绕x轴旋转：axis0=2, axis1=1, axis2=0
 * 参考：https://github.com/evbernardes/quaternion_to_euler
 *
 * @param q 输入四元数。
 * @param axis0 第一个旋转轴编号
 * @param axis1 第二个旋转轴编号
 * @param axis2 第三个旋转轴编号
 * @param extrinsic 是否使用外旋顺序。
 * @return 欧拉角向量。
 */
Eigen::Vector3d eulers(Eigen::Quaterniond q, int axis0, int axis1, int axis2,
                       bool extrinsic = false);

/**
 * @brief 旋转矩阵转欧拉角。
 *
 * x = 0, y = 1, z = 2
 * e.g. 先绕z轴旋转，再绕y轴旋转，最后绕x轴旋转：axis0=2, axis1=1, axis2=0
 *
 * @param R 输入旋转矩阵。
 * @param axis0 第一个旋转轴编号，x = 0, y = 1, z = 2。
 * @param axis1 第二个旋转轴编号，x = 0, y = 1, z = 2。
 * @param axis2 第三个旋转轴编号，x = 0, y = 1, z = 2。
 * @param extrinsic 是否使用外旋顺序。
 * @return 欧拉角向量。
 */
Eigen::Vector3d eulers(Eigen::Matrix3d R, int axis0, int axis1, int axis2, bool extrinsic = false);

/**
 * @brief 欧拉角转旋转矩阵。
 * @param ypr yaw、pitch、roll 欧拉角。
 * @return 对应的旋转矩阵。
 */
Eigen::Matrix3d rotation_matrix(const Eigen::Vector3d& ypr);

/**
 * @brief 直角坐标系转球坐标系。
 * @param xyz 直角坐标向量。
 * @return yaw、pitch、distance 形式的球坐标向量。
 */
Eigen::Vector3d xyz2ypd(const Eigen::Vector3d& xyz);

/**
 * @brief 计算 xyz2ypd 对 xyz 的雅可比矩阵。
 * @param xyz 直角坐标向量。
 * @return 雅可比矩阵。
 */
Eigen::MatrixXd xyz2ypd_jacobian(const Eigen::Vector3d& xyz);

/**
 * @brief 球坐标系转直角坐标系。
 * @param ypd yaw、pitch、distance 形式的球坐标向量。
 * @return 直角坐标向量。
 */
Eigen::Vector3d ypd2xyz(const Eigen::Vector3d& ypd);

/**
 * @brief 计算 ypd2xyz 对 ypd 的雅可比矩阵。
 * @param ypd yaw、pitch、distance 形式的球坐标向量。
 * @return 雅可比矩阵。
 */
Eigen::MatrixXd ypd2xyz_jacobian(const Eigen::Vector3d& ypd);

/**
 * @brief 计算两个 steady_clock 时间点的时间差。
 * @param a 被减时间点。
 * @param b 减数时间点。
 * @return 时间差 a - b，单位为 s。
 */
double delta_time(const std::chrono::steady_clock::time_point& a,
                  const std::chrono::steady_clock::time_point& b);

/**
 * @brief 计算两个二维向量的夹角绝对值。
 * @param vec1 第一个二维向量。
 * @param vec2 第二个二维向量。
 * @return 向量夹角，范围为 0 到 pi。
 */
double get_abs_angle(const Eigen::Vector2d& vec1, const Eigen::Vector2d& vec2);

/**
 * @brief 返回输入值的平方。
 * @tparam T 输入值类型。
 * @param a 输入值。
 * @return a 的平方。
 */
template <typename T> T square(T const& a) {
    return a * a;
};

/**
 * @brief 将输入值限制在闭区间 [min, max] 内。
 * @param input 输入值。
 * @param min 下限。
 * @param max 上限。
 * @return 限幅后的值。
 */
double limit_min_max(double input, double min, double max);
} // namespace tools

#endif // TOOLS__MATH_TOOLS_HPP
