#include <fmt/core.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/cboard//cboard.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "app/auto_aim/yolo.hpp"
#include "app/auto_aim/solver.hpp"
#include "app/predictor/aimer.hpp"
#include "app/decision/decider.hpp"
#include "app/decision/shooter.hpp"
#include "app/tracker/tracker.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;

// 启动参数：传入 TOML 配置文件所在目录，默认读取 configs 目录。
const std::string keys =
  "{help h usage ? |         | 输出命令行参数说明}"
  "{@config-dir    | configs | TOML配置文件目录，例如 configs }";

int main(int argc, char * argv[])
{
  // 工具模块：退出控制、调试绘图和数据录制。
  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  const auto config_dir = cli.get<std::string>(0);
  const auto camera_config = config_dir + "/camera.toml";
  const auto vision_config = config_dir + "/vision.toml";
  const auto game_config = config_dir + "/game.toml";
  const auto serial_config = config_dir + "/serial.toml";

  // 硬件接口：控制板、主相机、后置相机和两个 USB 辅助相机。
  io::CBoard cboard(serial_config);
  io::Camera camera(camera_config);
  io::Camera back_camera(camera_config);
  io::USBCamera usbcam1("video0", camera_config);
  io::USBCamera usbcam2("video2", camera_config);

  // 自瞄模块：检测、解算、跟踪、瞄准和射击判断。
  auto_aim::YOLO yolo(vision_config, false);
  auto_aim::Solver solver(camera_config);
  auto_aim::Tracker tracker(vision_config, solver);
  auto_aim::Aimer aimer(vision_config);
  auto_aim::Shooter shooter(vision_config);

  // 决策模块：目标过滤、优先级排序和丢目标后的全向感知决策。
  omniperception::Decider decider(game_config);

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  io::Command last_command;

  while (!exiter.exit()) {
    // 读取主相机图像，并根据图像时间戳获取 IMU 姿态。
    camera.read(img, timestamp);
    Eigen::Quaterniond q = cboard.imu_at(timestamp - 1ms);
    // recorder.record(img, q, timestamp);

    // 设置云台到世界坐标系的旋转矩阵，用于空间解算。
    solver.set_R_gimbal2world(q);
    Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    // 主相机检测装甲板，并结合比赛状态过滤不可攻击目标。
    auto armors = yolo.detect(img);
    decider.armor_filter(armors);
    decider.set_priority(armors);

    // 跟踪目标，得到当前可用于自瞄的目标列表。
    auto targets = tracker.track(armors, timestamp);

    io::Command command{false, false, 0, 0};

    // 丢目标时使用辅助相机进行全向感知，否则执行正常自瞄。
    if (tracker.state() == "lost")
      command = decider.decide(yolo, gimbal_pos, usbcam1, usbcam2, back_camera);
    else
      command = aimer.aim(targets, timestamp, cboard.bullet_speed, cboard.shoot_mode);

    // 统一判断是否开火，并发送最终控制命令。
    command.shoot = shooter.shoot(command, aimer, targets, gimbal_pos);
    cboard.send(command);
  }

  return 0;
}