/**
 * @file usbcamera.cpp
 * @brief 实现基于OpenCVVideoCapture的USB相机驱动。
 *
 * 该文件使用项目TOML工具读取相机参数，并把普通USB相机包装成带后台采集和时间戳的输入源。
 */

#include "usbcamera.hpp"

#include <stdexcept>
#include <string>

#include "tools/logger.hpp"
#include "tools/toml.hpp"

using namespace std::chrono_literals;

namespace io {

USBCamera::USBCamera(const std::string& open_name, const std::string& config_path)
    : open_name_(open_name), sharpness_(0), open_count_(0), quit_(false), ok_(false), queue_(1) {
    // USB相机参数错误通常只会表现为取图异常，因此在构造阶段提前读取并校验
    const auto config = tools::load(config_path);

    // 逐项读取V4L参数，后续open()统一写入VideoCapture
    image_width_ = tools::read<double>(config, "image_width");
    image_height_ = tools::read<double>(config, "image_height");
    usb_exposure_ = tools::read<double>(config, "usb_exposure");
    usb_frame_rate_ = tools::read<double>(config, "usb_frame_rate");
    usb_gamma_ = tools::read<double>(config, "usb_gamma");
    usb_gain_ = tools::read<double>(config, "usb_gain");

    // 构造阶段先打开一次，守护线程负责后续恢复
    try_open();

    daemon_thread_ = std::thread{[this] {
        while (!quit_) {
            std::this_thread::sleep_for(100ms);

            // 采集正常时无需干预VideoCapture
            if (ok_) {
                continue;
            }

            if (open_count_ > 20) {
                LOG_WARN("USBCAMERA", "Give up to open {} USB camera", this->device_name);
                // 超过重试次数后主动退出，避免设备不存在时无限重连
                quit_ = true;

                {
                    std::lock_guard<std::mutex> lock(cap_mutex_);
                    close();
                }

                if (capture_thread_.joinable()) {
                    LOG_WARN("USBCAMERA", "Stopping capture thread");
                    capture_thread_.join();
                }

                break;
            }

            if (capture_thread_.joinable()) {
                capture_thread_.join();
            }

            {
                std::lock_guard<std::mutex> lock(cap_mutex_);
                close();
            }

            // 关闭旧资源后重新打开设备
            try_open();
        }
    }};
}

USBCamera::~USBCamera() {
    // 析构先通知后台线程退出
    quit_ = true;

    {
        std::lock_guard<std::mutex> lock(cap_mutex_);
        close();
    }

    if (daemon_thread_.joinable()) {
        daemon_thread_.join();
    }
    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }

    LOG_INFO("USBCAMERA", "USBCamera destructed.");
}

cv::Mat USBCamera::read() {
    std::lock_guard<std::mutex> lock(cap_mutex_);
    // 同步read直接访问VideoCapture，需要持锁避免和采集线程并发
    if (!cap_.isOpened()) {
        LOG_WARN("USBCAMERA", "Failed to read {} USB camera", this->device_name);
        return cv::Mat();
    }

    cap_ >> img_;
    return img_;
}

void USBCamera::read(cv::Mat& img, std::chrono::steady_clock::time_point& timestamp) {
    // 队列read返回采集线程已经打好时间戳的帧
    CameraData data;
    queue_.pop(data);

    img = data.img;
    timestamp = data.timestamp;
}

void USBCamera::open() {
    std::lock_guard<std::mutex> lock(cap_mutex_);

    // OpenCV需要完整设备路径
    std::string true_device_name = "/dev/" + open_name_;
    cap_.open(true_device_name, cv::CAP_V4L);
    if (!cap_.isOpened()) {
        LOG_WARN("USBCAMERA", "Failed to open USB camera");
        return;
    }

    // 当前部署用sharpness区分左右相机
    sharpness_ = static_cast<int>(cap_.get(cv::CAP_PROP_SHARPNESS));

    // MJPG通常能显著降低USB带宽占用，双目USB相机更容易稳定跑高帧率
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap_.set(cv::CAP_PROP_FPS, usb_frame_rate_);
    cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
    cap_.set(cv::CAP_PROP_GAMMA, usb_gamma_);
    cap_.set(cv::CAP_PROP_GAIN, usb_gain_);

    if (sharpness_ == 2) {
        device_name = "left";
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, image_width_);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, image_height_);
        cap_.set(cv::CAP_PROP_EXPOSURE, usb_exposure_);
    } else if (sharpness_ == 3) {
        device_name = "right";
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, image_width_);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, image_height_);
        cap_.set(cv::CAP_PROP_EXPOSURE, usb_exposure_);
    } else {
        device_name = open_name_;
    }

    LOG_INFO("USBCAMERA", "{} USBCamera opened", device_name);
    LOG_INFO("USBCAMERA", "USBCamera fps:{}", cap_.get(cv::CAP_PROP_FPS));

    capture_thread_ = std::thread{[this] {
        // 采集线程启动后标记状态，供守护线程判断
        ok_ = true;
        std::this_thread::sleep_for(50ms);
        LOG_INFO("USBCAMERA", "[{} USB camera] capture thread started ", this->device_name);

        while (!quit_) {
            std::this_thread::sleep_for(1ms);

            cv::Mat img;
            bool success;
            {
                std::lock_guard<std::mutex> lock(cap_mutex_);
                if (!cap_.isOpened()) {
                    break;
                }
                // 读帧必须在锁内，避免close()同时释放设备
                success = cap_.read(img);
            }

            if (!success) {
                LOG_WARN("USBCAMERA", "Failed to read frame, exiting capture thread");
                break;
            }

            // 时间戳记录在成功取图后，作为软件采集时间
            auto timestamp = std::chrono::steady_clock::now();
            // 成功帧入队，容量1会优先保留新帧
            queue_.push({img, timestamp});
        }

        ok_ = false;
    }};
}

void USBCamera::try_open() {
    try {
        // open内部负责启动采集线程
        open();
        open_count_++;
    } catch (const std::exception& e) {
        LOG_WARN("USBCAMERA", "{}", e.what());
    }
}

void USBCamera::close() {
    if (cap_.isOpened()) {
        cap_.release();
        LOG_INFO("USBCAMERA", "USB camera released.");
    }
}

} // namespace io
