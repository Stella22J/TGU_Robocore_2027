/**
 * @file hikrobot.cpp
 * @brief 实现HikRobot工业相机驱动。
 *
 * 该文件封装MVS SDK取图、Bayer转RGB和异常恢复流程，使上层只面对统一CameraBase接口。
 */

#include "hikrobot.hpp"

#include <libusb-1.0/libusb.h>

#include <unordered_map>

#include "tools/logger.hpp"

using namespace std::chrono_literals;

namespace io {

HikRobot::HikRobot(double exposure_ms, double gain, const std::string& vid_pid)
    : exposure_us_(exposure_ms * 1e3), gain_(gain), daemon_quit_(false), handle_(nullptr),
      capturing_(false), capture_quit_(false), queue_(1), vid_(-1), pid_(-1) {
    // VID/PID只用于掉线恢复，不影响SDK枚举流程
    set_vid_pid(vid_pid);

    // libusb只服务于异常恢复，初始化失败不阻止SDK先尝试取图
    if (libusb_init(NULL)) {
        tools::logger()->warn("Unable to init libusb!");
    }

    daemon_thread_ = std::thread{[this] {
        tools::logger()->info("HikRobot's daemon thread started.");

        // 守护线程负责首次启动和异常重启
        capture_start();

        while (!daemon_quit_) {
            std::this_thread::sleep_for(100ms);
            if (capturing_) {
                continue;
            }

            // 完整重建SDK状态比局部重试更可靠，尤其是USB相机掉线后
            capture_stop();
            reset_usb();
            capture_start();
        }

        capture_stop();
        tools::logger()->info("HikRobot's daemon thread stopped.");
    }};
}

HikRobot::~HikRobot() {
    daemon_quit_ = true;

    if (daemon_thread_.joinable()) {
        daemon_thread_.join();
    }

    tools::logger()->info("HikRobot destructed.");
}

void HikRobot::read(cv::Mat& img, std::chrono::steady_clock::time_point& timestamp) {
    // 从采集线程队列取出最新图像
    CameraData data;
    queue_.pop(data);

    img = data.img;
    timestamp = data.timestamp;
}

void HikRobot::capture_start() {
    // 每次启动前先复位状态，避免继承上一次异常状态
    capturing_ = false;
    capture_quit_ = false;

    unsigned int ret;
    MV_CC_DEVICE_INFO_LIST device_list;

    // 通过MVS SDK枚举USB相机
    ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_EnumDevices failed: {:#x}", ret);
        return;
    }

    if (device_list.nDeviceNum == 0) {
        tools::logger()->warn("Not found camera!");
        return;
    }

    // 目前只取第一台SDK枚举设备，多相机筛选应在后续配置中扩展
    ret = MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[0]);
    if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_CreateHandle failed: {:#x}", ret);
        return;
    }

    // 句柄创建成功后才能打开设备并设置参数
    ret = MV_CC_OpenDevice(handle_);
    if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_OpenDevice failed: {:#x}", ret);
        return;
    }

    // 关闭曝光和增益自动模式，保证视觉算法输入亮度可控
    set_enum_value("BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
    set_enum_value("ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    set_enum_value("GainAuto", MV_GAIN_MODE_OFF);
    set_float_value("ExposureTime", exposure_us_);
    set_float_value("Gain", gain_);
    MV_CC_SetFrameRate(handle_, 150);

    // 开始SDK取流后再启动本地采集线程
    ret = MV_CC_StartGrabbing(handle_);
    if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_StartGrabbing failed: {:#x}", ret);
        return;
    }

    capture_thread_ = std::thread{[this] {
        tools::logger()->info("HikRobot's capture thread started.");

        capturing_ = true;

        MV_FRAME_OUT raw;
        MV_CC_PIXEL_CONVERT_PARAM cvt_param;

        while (!capture_quit_) {
            std::this_thread::sleep_for(1ms);

            unsigned int ret;
            unsigned int nMsec = 100;

            // SDK返回的buffer必须在处理后显式释放
            ret = MV_CC_GetImageBuffer(handle_, &raw, nMsec);
            if (ret != MV_OK) {
                tools::logger()->warn("MV_CC_GetImageBuffer failed: {:#x}", ret);
                break;
            }

            // 时间戳记录在拿到buffer后，作为软件采集时间
            auto timestamp = std::chrono::steady_clock::now();

            // 先包装SDK缓冲区，随后cvtColor会复制到OpenCV自有内存
            cv::Mat img(cv::Size(raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight), CV_8U,
                        raw.pBufAddr);

            // 保留SDK转换参数，便于后续从OpenCV转换切回SDK转换时复用
            cvt_param.nWidth = raw.stFrameInfo.nWidth;
            cvt_param.nHeight = raw.stFrameInfo.nHeight;
            cvt_param.pSrcData = raw.pBufAddr;
            cvt_param.nSrcDataLen = raw.stFrameInfo.nFrameLen;
            cvt_param.enSrcPixelType = raw.stFrameInfo.enPixelType;
            cvt_param.pDstBuffer = img.data;
            cvt_param.nDstBufferSize = img.total() * img.elemSize();
            cvt_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

            const auto& frame_info = raw.stFrameInfo;
            auto pixel_type = frame_info.enPixelType;

            cv::Mat dst_image;

            // 根据Bayer排列选择OpenCV转换规则
            const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> type_map = {
                {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB},
                {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
                {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB},
                {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB}};

            // 使用OpenCV转换后图像拥有独立内存，释放SDK缓冲区不会影响队列中的帧
            cv::cvtColor(img, dst_image, type_map.at(pixel_type));
            img = dst_image;

            // 转换完成后入队，保证队列中的图像不依赖SDKbuffer生命周期
            queue_.push({img, timestamp});

            ret = MV_CC_FreeImageBuffer(handle_, &raw);
            if (ret != MV_OK) {
                tools::logger()->warn("MV_CC_FreeImageBuffer failed: {:#x}", ret);
                break;
            }
        }

        capturing_ = false;
        tools::logger()->info("HikRobot's capture thread stopped.");
    }};
}

void HikRobot::capture_stop() {
    // 先停止采集线程，再释放SDK资源
    capture_quit_ = true;

    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }

    if (handle_ == nullptr) {
        return;
    }

    unsigned int ret;

    ret = MV_CC_StopGrabbing(handle_);
    if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_StopGrabbing failed: {:#x}", ret);
        return;
    }

    ret = MV_CC_CloseDevice(handle_);
    if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_CloseDevice failed: {:#x}", ret);
        return;
    }

    ret = MV_CC_DestroyHandle(handle_);
    if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_DestroyHandle failed: {:#x}", ret);
        return;
    }

    handle_ = nullptr;
}

void HikRobot::set_float_value(const std::string& name, double value) {
    unsigned int ret = MV_CC_SetFloatValue(handle_, name.c_str(), value);

    if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_SetFloatValue(\"{}\", {}) failed: {:#x}", name, value, ret);
    }
}

void HikRobot::set_enum_value(const std::string& name, unsigned int value) {
    unsigned int ret = MV_CC_SetEnumValue(handle_, name.c_str(), value);

    if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_SetEnumValue(\"{}\", {}) failed: {:#x}", name, value, ret);
    }
}

void HikRobot::set_vid_pid(const std::string& vid_pid) {
    // 解析十六进制VID/PID字符串
    auto index = vid_pid.find(':');
    if (index == std::string::npos) {
        tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
        return;
    }

    auto vid_str = vid_pid.substr(0, index);
    auto pid_str = vid_pid.substr(index + 1);

    try {
        vid_ = std::stoi(vid_str, 0, 16);
        pid_ = std::stoi(pid_str, 0, 16);
    } catch (const std::exception&) {
        tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
    }
}

void HikRobot::reset_usb() const {
    if (vid_ == -1 || pid_ == -1) {
        return;
    }

    // 参考usb-reset实现，直接复位设备比等待内核恢复更快
    auto handle = libusb_open_device_with_vid_pid(NULL, vid_, pid_);
    if (!handle) {
        tools::logger()->warn("Unable to open usb!");
        return;
    }

    if (libusb_reset_device(handle)) {
        tools::logger()->warn("Unable to reset usb!");
    } else {
        tools::logger()->info("Reset usb successfully :)");
    }

    libusb_close(handle);
}

} // namespace io
