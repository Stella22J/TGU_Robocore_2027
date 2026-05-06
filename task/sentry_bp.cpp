#include <fmt/core.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/cboard//cboard.hpp"
#include "io/ros2/ros2.hpp"
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

const std::string keys =
  "{help h usage ? |         | 输出命令行参数说明}"
  "{@config-dir    | configs | TOML配置文件目录 }";

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  auto config_dir = cli.get<std::string>(0);
  auto camera_config = config_dir + "/camera.toml";
  auto vision_config = config_dir + "/vision.toml";
  auto game_config = config_dir + "/game.toml";
  auto serial_config = config_dir + "/serial.toml";

  io::ROS2 ros2;
  io::CBoard cboard(serial_config);
  io::Camera camera(camera_config);
  io::Camera back_camera(camera_config);

  auto_aim::YOLO yolo(vision_config, false);
  auto_aim::Solver solver(camera_config);
  auto_aim::Tracker tracker(vision_config, solver);
  auto_aim::Aimer aimer(vision_config);
  auto_aim::Shooter shooter(vision_config);

  omniperception::Decider decider(game_config);

  cv::Mat img;

  std::chrono::steady_clock::time_point timestamp;
  io::Command last_command;

  while (!exiter.exit()) {
    camera.read(img, timestamp);
    Eigen::Quaterniond q = cboard.imu_at(timestamp - 1ms);
    // recorder.record(img, q, timestamp);

    /// 自瞄核心逻辑
    solver.set_R_gimbal2world(q);

    Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    auto armors = yolo.detect(img);

    decider.get_invincible_armor(ros2.subscribe_enemy_status());

    decider.armor_filter(armors);

    // decider.get_auto_aim_target(armors, ros2.subscribe_autoaim_target());

    decider.set_priority(armors);

    auto targets = tracker.track(armors, timestamp);

    io::Command command{false, false, 0, 0};

    /// 全向感知逻辑
    if (tracker.state() == "lost")
      command = decider.decide(yolo, gimbal_pos, back_camera);
    else
      command = aimer.aim(targets, timestamp, cboard.bullet_speed, cboard.shoot_mode);

    /// 发射逻辑
    command.shoot = shooter.shoot(command, aimer, targets, gimbal_pos);

    cboard.send(command);

    /// ROS2通信
    Eigen::Vector4d target_info = decider.get_target_info(armors, targets);

    ros2.publish(target_info);
  }
  return 0;
}