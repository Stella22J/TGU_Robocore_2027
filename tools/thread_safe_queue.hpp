/**
 * @file thread_safe_queue.hpp
 * @brief 通用线程安全有界队列模板。
 */

#ifndef TOOLS__THREAD_SAFE_QUEUE_HPP
#define TOOLS__THREAD_SAFE_QUEUE_HPP

#include <condition_variable>
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>

namespace tools {
/**
 * @brief 通用线程安全有界队列。
 * @tparam T 队列元素类型。
 * @tparam PopWhenFull 队列满时是否弹出旧元素。
 */
template <typename T, bool PopWhenFull = false> class ThreadSafeQueue {
  public:
    /**
     * @brief 创建线程安全有界队列。
     * @param max_size 队列最大容量。
     * @param full_handler 队列满且不弹出旧元素时调用的处理函数。
     */
    ThreadSafeQueue(
        size_t max_size, std::function<void(void)> full_handler = [] {})
        : max_size_(max_size), full_handler_(full_handler) {}

    /**
     * @brief 向队列尾部压入元素。
     * @param value 待压入元素。
     */
    void push(const T& value) {
        std::unique_lock<std::mutex> lock(mutex_);

        if (queue_.size() >= max_size_) {
            if (PopWhenFull) {
                queue_.pop();
            } else {
                full_handler_();
                return;
            }
        }

        queue_.push(value);
        not_empty_condition_.notify_all();
    }

    /**
     * @brief 阻塞式弹出队首元素。
     * @param value 用于接收弹出元素的引用。
     */
    void pop(T& value) {
        std::unique_lock<std::mutex> lock(mutex_);

        not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });

        if (queue_.empty()) {
            std::cerr << "Error: Attempt to pop from an empty queue." << std::endl;
            return;
        }

        value = queue_.front();
        queue_.pop();
    }

    /**
     * @brief 阻塞式弹出并返回队首元素。
     * @return 弹出的队首元素。
     */
    T pop() {
        std::unique_lock<std::mutex> lock(mutex_);

        not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });

        T value = std::move(queue_.front());
        queue_.pop();
        return std::move(value);
    }

    /**
     * @brief 阻塞式读取队首元素。
     * @return 队首元素副本。
     */
    T front() {
        std::unique_lock<std::mutex> lock(mutex_);

        not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });

        return queue_.front();
    }

    /**
     * @brief 读取队尾元素。
     * @param value 用于接收队尾元素的引用。
     */
    void back(T& value) {
        std::unique_lock<std::mutex> lock(mutex_);

        if (queue_.empty()) {
            std::cerr << "Error: Attempt to access the back of an empty queue." << std::endl;
            return;
        }

        value = queue_.back();
    }

    /**
     * @brief 查询队列是否为空。
     * @return 队列为空返回 true，否则返回 false。
     */
    bool empty() {
        std::unique_lock<std::mutex> lock(mutex_);
        return queue_.empty();
    }

    /**
     * @brief 清空队列。
     */
    void clear() {
        std::unique_lock<std::mutex> lock(mutex_);
        while (!queue_.empty()) {
            queue_.pop();
        }
        not_empty_condition_.notify_all(); // 如果其他线程正在等待队列不为空，这样可以唤醒它们
    }

  private:
    std::queue<T> queue_;
    size_t max_size_;
    mutable std::mutex mutex_;
    std::condition_variable not_empty_condition_;
    std::function<void(void)> full_handler_;
};

} // namespace tools

#endif // TOOLS__THREAD_SAFE_QUEUE_HPP