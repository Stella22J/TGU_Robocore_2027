/**
 * @file mindvision.cpp
 * @brief 实现MindVision工业相机驱动。
 *
 * 该文件封装MindVisionSDK初始化、参数设置、取图线程和USB异常恢复流程。
 */

#include "mindvision.hpp"

#include <libusb-1.0/libusb.h>

#include <stdexcept>

#include "tools/logger.hpp"

using namespace std::chrono_literals;

namespace io {

MindVision::MindVision(double exposure_ms, double gamma, const std::string& vid_pid)
    : exposure_ms_(exposure_ms), gamma_(gamma), handle_(-1), quit_(false), ok_(false), queue_(1),
      vid_(-1), pid_(-1) {
    // VID/PID只用于掉线恢复，不影响SDK枚举流程
    set_vid_pid(vid_pid);

    // libusb只用于异常恢复，失败时仍允许SDK先尝试打开相机
    if (libusb_init(NULL)) {
        LOG_WARN("MINDVISION", "Unable to init libusb!");
    }

    // 构造阶段先打开一次，守护线程负责后续失败重试
    try_open();

    daemon_thread_ = std::thread{[this] {
        while (!quit_) {
            std::this_thread::sleep_for(100ms);
            // 采集正常时守护线程只保持低频检查
            if (ok_) {
                continue;
            }

            if (capture_thread_.joinable()) {
                capture_thread_.join();
            }

            // 掉线后完整重建相机状态，比只重试取图更稳妥
            // 异常恢复采用完整关闭和重开流程
            close();
            reset_usb();
            try_open();
        }
    }};
}

MindVision::~MindVision() {
    quit_ = true;

    if (daemon_thread_.joinable()) {
        daemon_thread_.join();
    }
    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }

    close();

    LOG_INFO("MINDVISION", "Mindvision destructed.");
}

void MindVision::read(cv::Mat& img, std::chrono::steady_clock::time_point& timestamp) {
    // 从采集线程队列取出一帧图像
    CameraData data;
    queue_.pop(data);

    img = data.img;
    timestamp = data.timestamp;
}

void MindVision::open() {
    int camera_num = 1;
    tSdkCameraDevInfo camera_info_list;
    tSdkCameraCapbility camera_capbility;

    // SDK初始化放在open中，重启相机时也能恢复SDK状态
    CameraSdkInit(1);
    // 当前驱动使用SDK枚举到的第一台相机
    CameraEnumerateDevice(&camera_info_list, &camera_num);

    if (camera_num == 0) {
        throw std::runtime_error("Not found camera!");
    }

    if (CameraInit(&camera_info_list, -1, -1, &handle_) != CAMERA_STATUS_SUCCESS) {
        throw std::runtime_error("Failed to init camera!");
    }

    // 能力信息用于确定输出图像尺寸
    CameraGetCapability(handle_, &camera_capbility);

    // 当前项目使用最大分辨率，避免ROI变化时上层图像尺寸不稳定
    width_ = camera_capbility.sResolutionRange.iWidthMax;
    height_ = camera_capbility.sResolutionRange.iHeightMax;

    // 关闭自动曝光，保证图像亮度变化由配置控制
    CameraSetAeState(handle_, FALSE);
    CameraSetExposureTime(handle_, exposure_ms_ * 1e3);
    CameraSetGamma(handle_, gamma_ * 1e2);
    // 直接输出BGR8，减少上层OpenCV处理成本
    CameraSetIspOutFormat(handle_, CAMERA_MEDIA_TYPE_BGR8);
    CameraSetTriggerMode(handle_, 0);
    CameraSetFrameSpeed(handle_, 1);
    // SDK开始采集后启动本地取图线程
    CameraPlay(handle_);

    capture_thread_ = std::thread{[this] {
        tSdkFrameHead head;
        BYTE* raw;

        ok_ = true;

        while (!quit_) {
            std::this_thread::sleep_for(1ms);

            auto img = cv::Mat(height_, width_, CV_8UC3);
            // 获取SDK原始buffer，随后交给CameraImageProcess转换
            auto status = CameraGetImageBuffer(handle_, &head, &raw, 100);
            // 时间戳记录在取到buffer后，作为软件采集时间
            auto timestamp = std::chrono::steady_clock::now();

            if (status != CAMERA_STATUS_SUCCESS) {
                LOG_WARN("MINDVISION", "Camera dropped!");
                ok_ = false;
                break;
            }

            // SDK按前面设置的BGR8格式写入OpenCV内存
            CameraImageProcess(handle_, raw, img.data, &head);
            CameraReleaseImageBuffer(handle_, raw);

            // 图像处理完成后入队供read()消费
            queue_.push({img, timestamp});
        }
    }};

    LOG_INFO("MINDVISION", "Mindvision opened.");
}

void MindVision::try_open() {
    try {
        // 打开失败由异常路径记录，守护线程后续继续重试
        open();
    } catch (const std::exception& e) {
        LOG_WARN("MINDVISION", "{}", e.what());
    }
}

void MindVision::close() {
    if (handle_ == -1) {
        return;
    }

    // 释放SDK句柄，后续重连会重新初始化
    CameraUnInit(handle_);
    handle_ = -1;
}

void MindVision::set_vid_pid(const std::string& vid_pid) {
    // 解析十六进制VID/PID字符串
    auto index = vid_pid.find(':');
    if (index == std::string::npos) {
        LOG_WARN("MINDVISION", "Invalid vid_pid: \"{}\"", vid_pid);
        return;
    }

    auto vid_str = vid_pid.substr(0, index);
    auto pid_str = vid_pid.substr(index + 1);

    try {
        vid_ = std::stoi(vid_str, 0, 16);
        pid_ = std::stoi(pid_str, 0, 16);
    } catch (const std::exception&) {
        LOG_WARN("MINDVISION", "Invalid vid_pid: \"{}\"", vid_pid);
    }
}

void MindVision::reset_usb() const {
    if (vid_ == -1 || pid_ == -1) {
        return;
    }

    // 参考usb-reset实现，直接复位设备比等待内核恢复更快
    auto handle = libusb_open_device_with_vid_pid(NULL, vid_, pid_);
    if (!handle) {
        LOG_WARN("MINDVISION", "Unable to open usb!");
        return;
    }

    if (libusb_reset_device(handle)) {
        LOG_WARN("MINDVISION", "Unable to reset usb!");
    } else {
        LOG_INFO("MINDVISION", "Reset usb successfully :)");
    }

    libusb_close(handle);
}

} // namespace io
