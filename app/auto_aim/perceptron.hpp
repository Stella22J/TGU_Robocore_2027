/**
 * @file perceptron.hpp
 * @brief 并行全向感知接口，向上层提供多相机检测队列
 */

#ifndef OMNIPERCEPTION__PERCEPTRON_HPP
#define OMNIPERCEPTION__PERCEPTRON_HPP

#include <chrono>
#include <list>
#include <memory>

#include "app/auto_aim/armor.hpp"
#include "decider.hpp"
#include "detection.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tools/thread_pool.hpp"
#include "tools/thread_safe_queue.hpp"

namespace omniperception {

/**
 * @brief 为全向感知相机并行运行YOLO推理，让侧后方目标能更早被发现
 *
 * 每个相机独占一个模型实例，避免多个工作线程共享OpenVINO推理状态而引入竞争和额外锁开销
 */
class Perceptron {
  public:
    /**
     * @brief 启动四个相机推理工作线程，每个线程独立读取图像和执行检测
     * @param usbcam1 第一路USB相机，通常对应一个固定安装方向
     * @param usbcam2 第二路USB相机，通常对应另一个固定安装方向
     * @param usbcam3 第三路USB相机，用于扩大全向感知覆盖角度
     * @param usbcam4 第四路USB相机，用于补足剩余视野盲区
     * @param config_path TOML配置文件路径，用于初始化检测器和相机角度模型
     */
    Perceptron(io::USBCamera* usbcma1, io::USBCamera* usbcam2, io::USBCamera* usbcam3,
               io::USBCamera* usbcam4, const std::string& config_path);

    /**
     * @brief 停止所有推理线程，避免析构时相机或模型资源仍被后台访问
     */
    ~Perceptron();

    /**
     * @brief 取出当前队列中已经完成的检测结果，主循环可据此做目标切换决策
     * @return 当前检测队列的快照，调用后队列中的旧结果会被取出
     */
    std::vector<DetectionResult> get_detection_queue();

    /**
     * @brief 单个相机的工作循环，持续读取图像、推理并把结果推入队列
     * @param cam 当前工作线程绑定的相机指针，线程只读取这一路图像
     * @param yolo_parallel 当前工作线程独占的YOLO检测器实例
     */
    void parallel_infer(io::USBCamera* cam, std::shared_ptr<auto_aim::YOLO>& yolo_parallel);

  private:
    std::vector<std::thread> threads_;
    tools::ThreadSafeQueue<DetectionResult> detection_queue_;

    std::shared_ptr<auto_aim::YOLO> yolo_parallel1_;
    std::shared_ptr<auto_aim::YOLO> yolo_parallel2_;
    std::shared_ptr<auto_aim::YOLO> yolo_parallel3_;
    std::shared_ptr<auto_aim::YOLO> yolo_parallel4_;

    Decider decider_;
    bool stop_flag_;
    mutable std::mutex mutex_;
    std::condition_variable condition_;
};

} // namespace omniperception
#endif