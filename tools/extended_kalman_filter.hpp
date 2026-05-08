/**
 * @file extended_kalman_filter.hpp
 * @brief 扩展卡尔曼滤波器接口声明。
 */

#ifndef TOOLS__EXTENDED_KALMAN_FILTER_HPP
#define TOOLS__EXTENDED_KALMAN_FILTER_HPP

#include <Eigen/Dense>
#include <deque>
#include <functional>
#include <map>

namespace tools {
/**
 * @brief 扩展卡尔曼滤波器。
 */
class ExtendedKalmanFilter {
  public:
    // 当前状态向量。
    Eigen::VectorXd x;
    // 当前状态协方差矩阵。
    Eigen::MatrixXd P;

    /**
     * @brief 默认构造空滤波器。
     */
    ExtendedKalmanFilter() = default;

    /**
     * @brief 构造扩展卡尔曼滤波器。
     * @param x0 初始状态向量。
     * @param P0 初始状态协方差矩阵。
     * @param x_add 状态加法函数，用于支持非线性状态空间。
     */
    ExtendedKalmanFilter(
        const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0,
        std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)> x_add =
            [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) { return a + b; });

    /**
     * @brief 使用线性状态转移矩阵执行预测。
     * @param F 状态转移矩阵。
     * @param Q 过程噪声协方差矩阵。
     * @return 预测后的状态向量。
     */
    Eigen::VectorXd predict(const Eigen::MatrixXd& F, const Eigen::MatrixXd& Q);

    /**
     * @brief 使用自定义状态转移函数执行预测。
     * @param F 状态转移雅可比矩阵。
     * @param Q 过程噪声协方差矩阵。
     * @param f 状态转移函数。
     * @return 预测后的状态向量。
     */
    Eigen::VectorXd predict(const Eigen::MatrixXd& F, const Eigen::MatrixXd& Q,
                            std::function<Eigen::VectorXd(const Eigen::VectorXd&)> f);

    /**
     * @brief 使用线性观测矩阵执行更新。
     * @param z 观测向量。
     * @param H 观测矩阵。
     * @param R 观测噪声协方差矩阵。
     * @param z_subtract 观测残差计算函数。
     * @return 更新后的状态向量。
     */
    Eigen::VectorXd update(
        const Eigen::VectorXd& z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R,
        std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)> z_subtract =
            [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) { return a - b; });

    /**
     * @brief 使用自定义观测函数执行更新。
     * @param z 观测向量。
     * @param H 观测函数雅可比矩阵。
     * @param R 观测噪声协方差矩阵。
     * @param h 观测函数。
     * @param z_subtract 观测残差计算函数。
     * @return 更新后的状态向量。
     */
    Eigen::VectorXd update(
        const Eigen::VectorXd& z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R,
        std::function<Eigen::VectorXd(const Eigen::VectorXd&)> h,
        std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)> z_subtract =
            [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) { return a - b; });

    //卡方检验与残差调试数据。
    std::map<std::string, double> data; // 卡方检验数据
    // 最近窗口内 NIS 检验失败标记队列。
    std::deque<int> recent_nis_failures{0};
    // NIS 失败统计窗口大小。
    size_t window_size = 100;
    // 最近一次 NIS 值。
    double last_nis;

  private:
    Eigen::MatrixXd I;
    std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)> x_add;

    int nees_count_ = 0;
    int nis_count_ = 0;
    int total_count_ = 0;
};

} // namespace tools

#endif // TOOLS__EXTENDED_KALMAN_FILTER_HPP
