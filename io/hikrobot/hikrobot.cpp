/**
 * @file hikrobot.cpp
 * @brief HikRobot工业相机驱动类实现。
 *
 * 本文件实现HikRobot类，完成HikRobotUSB工业相机的初始化、参数配置、图像采集、图像格式转换、异常重启和USB重置。
 *
 * 程序整体流程：
 * 1. 构造HikRobot对象
 * 2. 解析USB、VID/PID
 * 3. 初始化libusb
 * 4. 启动守护线程daemon_thread_
 * 5. 守护线程调用capture_start() 启动相机
 * 6. capture_start()打开相机并启动capture_thread_
 * 7. capture_thread_循环获取图像buffer
 * 8. 将Bayer图像转换为RGB图像
 * 9. 将图像和时间戳写入线程安全队列
 * 10. 若采集线程异常退出，守护线程会尝试stop、reset、restart
 */

#include "hikrobot.hpp"

#include <libusb-1.0/libusb.h>
#include <unordered_map>

#include "tools/logger.hpp"

using namespace std::chrono_literals;

namespace io {

HikRobot::HikRobot(double exposure_ms, double gain, const std::string& vid_pid)
    : exposure_us_(exposure_ms * 1e3), gain_(gain), queue_(1), daemon_quit_(false), vid_(-1),
      pid_(-1) {
    // 解析USB、VID/PID，用于reset_usb()。
    set_vid_pid(vid_pid);

    // 初始化libusb。用于reset_usb()。
    if (libusb_init(NULL))
        tools::logger()->warn("Unable to init libusb!");

    /**
     * @brief 启动相机守护线程。
     *
     * 守护线程负责：
     * 1. 启动相机采集
     * 2. 周期性检查采集线程状态
     * 3. 如果采集线程异常退出，则停止相机、重置USB、重新启动采集
     * 4. 析构时安全停止采集
     */
    daemon_thread_ = std::thread{[this] {
        tools::logger()->info("HikRobot's daemon thread started.");

        // 首次启动相机采集。
        capture_start();

        while (!daemon_quit_) {
            std::this_thread::sleep_for(100ms);

            // 采集线程仍在正常工作。
            if (capturing_)
                continue;

            // 若采集线程停止或异常退出，则尝试完整重启相机。
            capture_stop();
            reset_usb();
            capture_start();
        }

        // 守护线程退出前，停止采集并释放相机资源。
        capture_stop();

        tools::logger()->info("HikRobot's daemon thread stopped.");
    }};
}

HikRobot::~HikRobot() {
    // 通知守护线程退出。
    daemon_quit_ = true;

    // 等待守护线程结束。
    if (daemon_thread_.joinable())
        daemon_thread_.join();

    tools::logger()->info("HikRobot destructed.");
}

void HikRobot::read(cv::Mat& img, std::chrono::steady_clock::time_point& timestamp) {
    CameraData data;

    // 从图像队列中取出一帧数据；若队列为空，阻塞。
    queue_.pop(data);

    img = data.img;
    timestamp = data.timestamp;
}

void HikRobot::capture_start() {
    // 重置采集状态。
    capturing_ = false;
    capture_quit_ = false;

    unsigned int ret;

    // 枚举HikRobot USB相机设备。
    MV_CC_DEVICE_INFO_LIST device_list;
    ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_EnumDevices failed: {:#x}", ret);
        return;
    }

    // 未找到可用相机
    if (device_list.nDeviceNum == 0) {
        tools::logger()->warn("Not found camera!");
        return;
    }

    // 创建相机句柄
    ret = MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[0]);
    if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_CreateHandle failed: {:#x}", ret);
        return;
    }

    // 打开相机设备
    ret = MV_CC_OpenDevice(handle_);
    if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_OpenDevice failed: {:#x}", ret);
        return;
    }

    // 设置相机参数(白平衡、连续自动)
    set_enum_value("BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);

    // 曝光和增益(关闭自动模式，使用手动设置)
    set_enum_value("ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    set_enum_value("GainAuto", MV_GAIN_MODE_OFF);
    set_float_value("ExposureTime", exposure_us_);
    set_float_value("Gain", gain_);

    // 设置相机帧率
    MV_CC_SetFrameRate(handle_, 150);

    // 开始取流
    ret = MV_CC_StartGrabbing(handle_);
    if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_StartGrabbing failed: {:#x}", ret);
        return;
    }

    /**
     * @brief 启动图像采集线程。
     *
     * 采集线程循环执行：
     * 1. 从HikRobot SDK获取图像 buffer
     * 2. 记录当前steady_clock时间戳
     * 3. 将原始buffer包装成cv::Mat
     * 4. 根据Bayer像素格式转换为RGB图像
     * 5. 将图像和时间戳写入队列
     * 6. 释放SDK图像 buffer。
     */
    capture_thread_ = std::thread{[this] {
        tools::logger()->info("HikRobot's capture thread started.");

        capturing_ = true;

        MV_FRAME_OUT raw;
        MV_CC_PIXEL_CONVERT_PARAM cvt_param;

        // 主循环
        while (!capture_quit_) {
            std::this_thread::sleep_for(1ms);

            unsigned int ret;
            unsigned int nMsec = 100;

            // 从相机SDK获取一帧图像buffer
            ret = MV_CC_GetImageBuffer(handle_, &raw, nMsec);
            if (ret != MV_OK) {
                tools::logger()->warn("MV_CC_GetImageBuffer failed: {:#x}", ret);
                break;
            }

            // 记录成功获取图像buffer的时间
            auto timestamp = std::chrono::steady_clock::now();

            // 将SDK原始图像buffer包装成单通道CV::Mat。
            cv::Mat img(cv::Size(raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight), CV_8U,
                        raw.pBufAddr);

            // 以下参数用于HikRobot SDK自带的像素格式转换。
            cvt_param.nWidth = raw.stFrameInfo.nWidth;
            cvt_param.nHeight = raw.stFrameInfo.nHeight;

            cvt_param.pSrcData = raw.pBufAddr;
            cvt_param.nSrcDataLen = raw.stFrameInfo.nFrameLen;
            cvt_param.enSrcPixelType = raw.stFrameInfo.enPixelType;

            cvt_param.pDstBuffer = img.data;
            cvt_param.nDstBufferSize = img.total() * img.elemSize();
            cvt_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

            // 使用HikRobot SDK进行像素格式转换
            // ret = MV_CC_ConvertPixelType(handle_, &cvt_param);

            // 获取当前帧的像素格式
            const auto& frame_info = raw.stFrameInfo;
            auto pixel_type = frame_info.enPixelType;

            cv::Mat dst_image;

            // HikRobot Bayer8像素格式到Bayer转RGB规则的映射
            const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> type_map = {
                {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB},
                {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
                {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB},
                {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB}};

            // 将Bayer单通道图像转换为RGB三通道图像
            cv::cvtColor(img, dst_image, type_map.at(pixel_type));

            // 转换后img引用dst_image的数据
            img = dst_image;

            // 将图像和时间戳写入线程安全队列，供read()读取。
            queue_.push({img, timestamp});

            // 释放HikRobot SDK图像buffer。
            ret = MV_CC_FreeImageBuffer(handle_, &raw);
            if (ret != MV_OK) {
                tools::logger()->warn("MV_CC_FreeImageBuffer failed: {:#x}", ret);
                break;
            }
        }

        // 采集循环退出，通知守护线程当前采集已停止
        capturing_ = false;

        tools::logger()->info("HikRobot's capture thread stopped.");
    }};
}

void HikRobot::capture_stop() {
    // 通知采集线程退出
    capture_quit_ = true;

    // 等待采集线程结束
    if (capture_thread_.joinable())
        capture_thread_.join();

    unsigned int ret;

    // 停止相机取流
    ret = MV_CC_StopGrabbing(handle_);
    if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_StopGrabbing failed: {:#x}", ret);
        return;
    }

    // 关闭相机设备
    ret = MV_CC_CloseDevice(handle_);
    if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_CloseDevice failed: {:#x}", ret);
        return;
    }

    // 销毁相机句柄
    ret = MV_CC_DestroyHandle(handle_);
    if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_DestroyHandle failed: {:#x}", ret);
        return;
    }
}

void HikRobot::set_float_value(const std::string& name, double value) {
    unsigned int ret;

    // 调用HikRobot SDK设置浮点参数
    ret = MV_CC_SetFloatValue(handle_, name.c_str(), value);

    if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_SetFloatValue(\"{}\", {}) failed: {:#x}", name, value, ret);
        return;
    }
}

void HikRobot::set_enum_value(const std::string& name, unsigned int value) {
    unsigned int ret;

    // 调用HikRobot SDK设置枚举参数
    ret = MV_CC_SetEnumValue(handle_, name.c_str(), value);

    if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_SetEnumValue(\"{}\", {}) failed: {:#x}", name, value, ret);
        return;
    }
}

void HikRobot::set_vid_pid(const std::string& vid_pid) {
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

void HikRobot::reset_usb() const {
    // VID/PID无效时，不执行reset_usb()
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