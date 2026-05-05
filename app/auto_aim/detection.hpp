/**
 * @file detection.hpp
 * @brief 全向感知检测结果模型，用于多相机感知线程和决策模块之间传递数据
 */

#ifndef OMNIPERCEPTION__DETECTION_HPP
#define OMNIPERCEPTION__DETECTION_HPP

#include <chrono>
#include <list>

#include "app/auto_aim/armor.hpp"

namespace omniperception {
// 一个识别结果可能包含多个armor,需要排序和过滤。armors, timestamp, delta_yaw, delta_pitch
/**
 * @brief 全向感知和决策逻辑共享的检测包，包含目标列表、时间戳和粗略云台偏角
 *
 * 角度字段让决策层在未完成完整三维位姿解算前也能先引导云台转向目标方向
 */
struct DetectionResult {
    std::list<auto_aim::Armor> armors;
    std::chrono::steady_clock::time_point timestamp;
    double delta_yaw;   // rad
    double delta_pitch; // rad

    // 拷贝赋值用于在线程队列中传递检测结果，显式保留可以避免遗漏时间戳和偏角
    DetectionResult& operator=(const DetectionResult& other) {
        if (this != &other) {
            armors = other.armors;
            timestamp = other.timestamp;
            delta_yaw = other.delta_yaw;
            delta_pitch = other.delta_pitch;
        }
        return *this;
    }
};
} // namespace omniperception

#endif