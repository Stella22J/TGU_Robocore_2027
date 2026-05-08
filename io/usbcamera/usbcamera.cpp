/**
 * @file usbcamera.cpp
 * @brief 实现基于OpenCVVideoCapture的USB相机驱动。
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
    const auto config = tools::load(config_path);

    image_width_ = tools::read<double>(config, "image_width");
    image_height_ = tools::read<double>(config, "image_height");
    usb_exposure_ = tools::read<double>(config, "usb_exposure");
    usb_frame_rate_ = tools::read<double>(config, "usb_frame_rate");
    usb_gamma_ = tools::read<double>(config, "usb_gamma");
    usb_gain_ = tools::read<double>(config, "usb_gain");

    try_open();

    daemon_thread_ = std::thread{[this] {
        while (!quit_) {
            std::this_thread::sleep_for(100ms);

            if (ok_) {
                continue;
            }

            if (open_count_ > 20) {
                LOG_WARN("USBCAMERA", "Give up to open {} USB camera", this->device_name);
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

            try_open();
        }
    }};
}

USBCamera::~USBCamera() {
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
    if (!cap_.isOpened()) {
        LOG_WARN("USBCAMERA", "Failed to read {} USB camera", this->device_name);
        return cv::Mat();
    }

    cap_ >> img_;
    return img_;
}

void USBCamera::read(cv::Mat& img, std::chrono::steady_clock::time_point& timestamp) {
    CameraData data;
    queue_.pop(data);

    img = data.img;
    timestamp = data.timestamp;
}

void USBCamera::open() {
    std::lock_guard<std::mutex> lock(cap_mutex_);

    std::string true_device_name = "/dev/" + open_name_;
    cap_.open(true_device_name, cv::CAP_V4L);
    if (!cap_.isOpened()) {
        LOG_WARN("USBCAMERA", "Failed to open USB camera");
        return;
    }

    sharpness_ = static_cast<int>(cap_.get(cv::CAP_PROP_SHARPNESS));

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
                success = cap_.read(img);
            }

            if (!success) {
                LOG_WARN("USBCAMERA", "Failed to read frame, exiting capture thread");
                break;
            }

            auto timestamp = std::chrono::steady_clock::now();
            queue_.push({img, timestamp});
        }

        ok_ = false;
    }};
}

void USBCamera::try_open() {
    try {
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
