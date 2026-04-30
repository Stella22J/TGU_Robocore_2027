/**
 * @file mindvision.cpp
 * @brief MindVision 工业相机驱动类实现。
 *
 * 本文件实现 MindVision 类，完成 MindVision 工业相机的初始化、
 * 参数配置、图像采集、图像转换、异常重启和 USB 重置。
 *
 * 程序整体流程：
 * 1. 构造MindVision对象
 * 2. 保存曝光和伽马参数
 * 3. 解析USB VID/PID
 * 4. 初始化libusb
 * 5. 调用try_open()尝试打开相机
 * 6. open()初始化MindVision SDK，枚举并打开相机
 * 7. 设置曝光、伽马、BGR8 输出、连续采集模式
 * 8. 启动capture_thread_采集线程
 * 9. 采集线程循环获取图像buffer，并转换为OpenCV图像
 * 10. 将图像和时间戳写入线程安全队列
 * 11. 若采集异常，daemon_thread_会尝试关闭相机、重置USB并重新打开。
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
    // 解析USB VID/PID
    set_vid_pid(vid_pid);

    // 初始化 libusb
    if (libusb_init(NULL))
        tools::logger()->warn("Unable to init libusb!");

    // 首次尝试打开相机
    try_open();

    /**
     * @brief 启动相机守护线程。
     *
     * 守护线程负责：
     * 1. 周期性检查采集状态ok_
     * 2. 如果ok_为true，说明采集线程正常工作;如果ok_为false，说明相机未打开或采集异常
     * 3. 异常时等待采集线程退出，关闭相机，重置 USB，并尝试重新打开相机
     */
    daemon_thread_ = std::thread{[this] {
        while (!quit_) {
            std::this_thread::sleep_for(100ms);

            // 相机采集正常
            if (ok_)
                continue;

            // 如果采集线程已退出但仍可join，则先回收线程资源
            if (capture_thread_.joinable())
                capture_thread_.join();

            // 异常恢复
            close();
            reset_usb();
            try_open();
        }
    }};
}

MindVision::~MindVision() {
    // 通知守护线程和采集线程退出
    quit_ = true;

    // 等待守护线程结束
    if (daemon_thread_.joinable())
        daemon_thread_.join();

    // 等待采集线程结束
    if (capture_thread_.joinable())
        capture_thread_.join();

    // 关闭相机并释放SDK资源
    close();

    tools::logger()->info("Mindvision destructed.");
}

void MindVision::read(cv::Mat& img, std::chrono::steady_clock::time_point& timestamp) {
    CameraData data;

    // 从图像队列中取出一帧数据；若队列为空，阻塞。
    queue_.pop(data);

    img = data.img;
    timestamp = data.timestamp;
}

void MindVision::open() {
    int camera_num = 1;
    tSdkCameraDevInfo camera_info_list;
    tSdkCameraCapbility camera_capbility;

    // 初始化MindVision SDK。
    CameraSdkInit(1);

    // 枚举相机设备
    CameraEnumerateDevice(&camera_info_list, &camera_num);

    // 没有找到可用相机时抛出异常
    if (camera_num == 0)
        throw std::runtime_error("Not found camera!");

    // 初始化相机并获取SDK句柄
    if (CameraInit(&camera_info_list, -1, -1, &handle_) != CAMERA_STATUS_SUCCESS)
        throw std::runtime_error("Failed to init camera!");

    // 获取相机能力信息，包括最大分辨率、像素格式能力等
    CameraGetCapability(handle_, &camera_capbility);

    // 当前实现使用相机最大分辨率创建OpenCV图像
    width_ = camera_capbility.sResolutionRange.iWidthMax;
    height_ = camera_capbility.sResolutionRange.iHeightMax;

    // 关闭自动曝光，使用手动曝光
    CameraSetAeState(handle_, FALSE);

    // 设置曝光时间。SDK接口通常使用微秒，因此由毫秒转换为微秒
    CameraSetExposureTime(handle_, exposure_ms_ * 1e3);

    // 设置伽马值。MindVision SDK通常使用 gamma * 100 的整数/浮点表示
    CameraSetGamma(handle_, gamma_ * 1e2);

    // 设置ISP输出格式为BGR8，便于直接写入CV_8UC3图像
    CameraSetIspOutFormat(handle_, CAMERA_MEDIA_TYPE_BGR8);

    // 设置为连续采集模式
    CameraSetTriggerMode(handle_, 0);

    // 设置为低帧率模式
    CameraSetFrameSpeed(handle_, 1);

    // 开始相机采集
    CameraPlay(handle_);

    /**
     * @brief 启动图像采集线程。
     *
     * 采集线程循环执行：
     * 1. 创建OpenCV输出图像
     * 2. 调用CameraGetImageBuffer()从SDK获取原始图像buffer
     * 3. 记录取到图像buffer的时间戳
     * 4. 调用CameraImageProcess()将原始图像转换为BGR8图像
     * 5. 调用CameraReleaseImageBuffer()释放SDK buffer
     * 6. 将图像和时间戳写入队列
     */
    capture_thread_ = std::thread{[this] {
        tSdkFrameHead head;
        BYTE* raw;

        // 标记当前采集状态正常
        ok_ = true;

        while (!quit_) {
            std::this_thread::sleep_for(1ms);

            // 创建BGR8输出图像
            auto img = cv::Mat(height_, width_, CV_8UC3);

            // 从相机SDK获取一帧原始图像buffer，超时时间100ms
            auto status = CameraGetImageBuffer(handle_, &head, &raw, 100);

            // 记录成功调用取图接口后的时间
            auto timestamp = std::chrono::steady_clock::now();

            // 取图失败
            if (status != CAMERA_STATUS_SUCCESS) {
                tools::logger()->warn("Camera dropped!");
                ok_ = false;
                break;
            }

            // 将原始图像buffer处理为前面设置的BGR8输出格式
            CameraImageProcess(handle_, raw, img.data, &head);

            // 释放SDK图像buffer
            CameraReleaseImageBuffer(handle_, raw);

            // 将图像和时间戳写入线程安全队列，供read()读取
            queue_.push({img, timestamp});
        }
    }};

    tools::logger()->info("Mindvision opened.");
}

void MindVision::try_open() {
    try {
        // 尝试打开相机，失败时由open()抛出异常。
        open();
    } catch (const std::exception& e) {
        // 打开失败只记录日志，不终止程序；守护线程后续会继续尝试恢复
        tools::logger()->warn("{}", e.what());
    }
}

void MindVision::close() {
    // handle_为-1表示当前没有有效相机句柄
    if (handle_ == -1)
        return;

    // 释放MindVision相机资源
    CameraUnInit(handle_);
}

void MindVision::set_vid_pid(const std::string& vid_pid) {
    // 查找VID和PID之间的分隔符
    auto index = vid_pid.find(':');
    if (index == std::string::npos) {
        tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
        return;
    }

    auto vid_str = vid_pid.substr(0, index);
    auto pid_str = vid_pid.substr(index + 1);

    try {
        // 按十六进制解析VID/PID
        vid_ = std::stoi(vid_str, 0, 16);
        pid_ = std::stoi(pid_str, 0, 16);
    } catch (const std::exception&) {
        tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
    }
}

void MindVision::reset_usb() const {
    // VID/PID无效时，不执行reset
    if (vid_ == -1 || pid_ == -1)
        return;

    // 参考实现：
    // https://github.com/ralight/usb-reset/blob/master/usb-reset.c

    // 根据VID/PID打开对应USB设备。
    auto handle = libusb_open_device_with_vid_pid(NULL, vid_, pid_);
    if (!handle) {
        tools::logger()->warn("Unable to open usb!");
        return;
    }

    // 执行USB设备重置。
    if (libusb_reset_device(handle))
        tools::logger()->warn("Unable to reset usb!");
    else
        tools::logger()->info("Reset usb successfully :)");

    // 关闭libusb设备句柄。
    libusb_close(handle);
}

} // namespace io