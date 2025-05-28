#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "picasso_arm.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PicassoArm>();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();

  return 0;
}