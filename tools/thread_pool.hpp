/**
 * @file thread_pool.hpp
 * @brief 多线程任务池与帧保序队列工具。
 */

#ifndef TOOLS__THREAD_POOL_HPP
#define TOOLS__THREAD_POOL_HPP

#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include "app/auto_aim/yolo.hpp"
#include "tools/logger.hpp"

namespace tools {
/**
 * @brief 待处理图像帧及其检测上下文。
 */
struct Frame {
    // 帧序号。
    int id;
    // 图像帧。
    cv::Mat img;
    //帧时间戳。
    std::chrono::steady_clock::time_point t;
    // 帧姿态四元数。
    Eigen::Quaterniond q;
    // 该帧检测到的装甲板列表。
    std::list<auto_aim::Armor> armors;
};

/**
 * @brief 创建多个 YOLO11 检测器实例。
 * @param config_path 检测器配置文件路径。
 * @param numebr 创建实例数量。
 * @param debug 是否启用调试模式。
 * @return YOLO11 检测器实例列表。
 */
inline std::vector<auto_aim::YOLO> create_yolo11s(const std::string& config_path, int numebr,
                                                  bool debug) {
    std::vector<auto_aim::YOLO> yolo11s;
    for (int i = 0; i < numebr; i++) {
        yolo11s.push_back(auto_aim::YOLO(config_path, debug));
    }
    return yolo11s;
}

/**
 * @brief 创建多个 YOLOv8 检测器实例。
 * @param config_path 检测器配置文件路径。
 * @param numebr 创建实例数量。
 * @param debug 是否启用调试模式。
 * @return YOLOv8 检测器实例列表。
 */
inline std::vector<auto_aim::YOLO> create_yolov8s(const std::string& config_path, int numebr,
                                                  bool debug) {
    std::vector<auto_aim::YOLO> yolov8s;
    for (int i = 0; i < numebr; i++) {
        yolov8s.push_back(auto_aim::YOLO(config_path, debug));
    }
    return yolov8s;
}

/**
 * @brief 按帧序号保序输出的线程安全队列。
 */
class OrderedQueue {
  public:
    /**
     * @brief 创建从帧序号 1 开始的保序队列。
     */
    OrderedQueue() : current_id_(1) {}

    /**
     * @brief 清空队列与缓存并销毁对象。
     */
    ~OrderedQueue() {
        {
            std::lock_guard<std::mutex> lock(mutex_);

            main_queue_ = std::queue<tools::Frame>();
            buffer_.clear();
            current_id_ = 0;
        }
        LOG_INFO("THREAD_POOL", "OrderedQueue destroyed, queue and buffer cleared.");
    }

    /**
     * @brief 将帧按序放入队列或缓存中。
     * @param item 待入队帧。
     */
    void enqueue(const tools::Frame& item) {
        std::lock_guard<std::mutex> lock(mutex_);

        if (item.id < current_id_) {
            LOG_WARN("THREAD_POOL", "small id");
            return;
        }

        if (item.id == current_id_) {
            main_queue_.push(item);
            current_id_++;

            auto it = buffer_.find(current_id_);
            while (it != buffer_.end()) {
                main_queue_.push(it->second);
                buffer_.erase(it);
                current_id_++;
                it = buffer_.find(current_id_);
            }

            if (main_queue_.size() >= 1) {
                cond_var_.notify_one();
            }
        } else {
            buffer_[item.id] = item;
        }
    }

    /**
     * @brief 阻塞式弹出下一帧。
     * @return 按序输出的下一帧。
     */
    tools::Frame dequeue() {
        std::unique_lock<std::mutex> lock(mutex_);

        cond_var_.wait(lock, [this]() { return !main_queue_.empty(); });

        tools::Frame item = main_queue_.front();
        main_queue_.pop();
        return item;
    }

    /**
     * @brief 非阻塞式尝试弹出下一帧。
     * @param item 用于接收弹出帧的引用。
     * @return 成功弹出返回 true，队列为空返回 false。
     */
    bool try_dequeue(tools::Frame& item) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (main_queue_.empty()) {
            return false;
        }
        item = main_queue_.front();
        main_queue_.pop();
        return true;
    }

    /**
     * @brief 获取主队列与缓存中的总帧数。
     * @return 总帧数。
     */
    size_t get_size() {
        return main_queue_.size() + buffer_.size();
    }

  private:
    std::queue<tools::Frame> main_queue_;
    std::unordered_map<int, tools::Frame> buffer_;
    int current_id_;
    std::mutex mutex_;
    std::condition_variable cond_var_;
};

/**
 * @brief 固定线程数任务池。
 */
class ThreadPool {
  public:
    /**
     * @brief 创建任务池并启动工作线程。
     * @param num_threads 工作线程数量。
     */
    ThreadPool(size_t num_threads) : stop(false) {
        for (size_t i = 0; i < num_threads; ++i) {
            workers.emplace_back([this] {
                while (true) {
                    std::function<void()> task;
                    {
                        std::unique_lock<std::mutex> lock(queue_mutex);
                        condition.wait(lock, [this] { return stop || !tasks.empty(); });
                        if (stop && tasks.empty()) {
                            return;
                        }
                        task = std::move(tasks.front());
                        tasks.pop();
                    }
                    task();
                }
            });
        }
    }

    /**
     * @brief 停止任务池并等待工作线程退出。
     */
    ~ThreadPool() {
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            stop = true;
            tasks = std::queue<std::function<void()>>();
        }
        condition.notify_all();
        for (std::thread& worker : workers) {
            if (worker.joinable()) {
                worker.join();
            }
        }
    }

    /**
     * @brief 添加任务到任务队列。
     * @tparam F 可调用对象类型。
     * @param f 待执行任务。
     */
    template <class F> void enqueue(F&& f) {
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            if (stop) {
                throw std::runtime_error("enqueue on stopped ThreadPool");
            }
            tasks.emplace(std::forward<F>(f));
        }
        condition.notify_one();
    }

  private:
    std::vector<std::thread> workers;        // 工作线程
    std::queue<std::function<void()>> tasks; // 任务队列
    std::mutex queue_mutex;                  // 任务队列互斥锁
    std::condition_variable condition;       // 条件变量，用于等待任务
    bool stop;                               // 是否停止线程池
};
} // namespace tools

#endif // TOOLS__THREAD_POOL_HPP
