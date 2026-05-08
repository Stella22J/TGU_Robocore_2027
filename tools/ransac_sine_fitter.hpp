/**
 * @file ransac_sine_fitter.hpp
 * @brief 基于 RANSAC 的正弦曲线拟合接口声明。
 */

#ifndef TOOLS__RANSAC_SINE_FITTER_HPP
#define TOOLS__RANSAC_SINE_FITTER_HPP

#include <Eigen/Dense>
#include <deque>
#include <iostream>
#include <random>
#include <vector>

namespace tools {

/**
 * @brief 基于 RANSAC 的正弦曲线拟合器。
 */
class RansacSineFitter {
  public:
    /**
     * @brief 正弦拟合结果。
     */
    struct Result {
        // 正弦幅值。
        double A = 0.0;
        // 角频率。
        double omega = 0.0;
        // 初相位。
        double phi = 0.0;
        // 直流偏置。
        double C = 0.0;
        // 内点数量。
        int inliers = 0;
    };
    // 当前最佳拟合结果。
    Result best_result_;

    /**
     * @brief 创建 RANSAC 正弦拟合器。
     * @param max_iterations 最大迭代次数。
     * @param threshold 内点判定阈值。
     * @param min_omega 搜索角频率下限。
     * @param max_omega 搜索角频率上限。
     */
    RansacSineFitter(int max_iterations, double threshold, double min_omega, double max_omega);

    /**
     * @brief 添加一组待拟合数据。
     * @param t 自变量。
     * @param v 观测值。
     */
    void add_data(double t, double v);

    /**
     * @brief 执行 RANSAC 正弦拟合并更新最佳结果。
     */
    void fit();

    /**
     * @brief 计算正弦模型函数值。
     * @param t 自变量。
     * @param A 幅值。
     * @param omega 角频率。
     * @param phi 初相位。
     * @param C 直流偏置。
     * @return 正弦模型函数值。
     */
    double sine_function(double t, double A, double omega, double phi, double C) {
        return A * std::sin(omega * t + phi) + C;
    }

  private:
    int max_iterations_;
    double threshold_;
    double min_omega_;
    double max_omega_;
    std::mt19937 gen_;
    std::deque<std::pair<double, double>> fit_data_;

    bool fit_partial_model(const std::vector<std::pair<double, double>>& sample, double omega,
                           Eigen::Vector3d& params);

    int evaluate_inliers(double A, double omega, double phi, double C);
};

} // namespace tools

#endif // TOOLS__RANSAC_SINE_FITTER_HPP
