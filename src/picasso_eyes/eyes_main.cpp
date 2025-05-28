#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "picasso_eyes.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PicassoEyes>();
  auto tf_node = std::make_shared<DynamicFrameBroadcaster>();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(tf_node);
  exec.spin();

  rclcpp::shutdown();

  return 0;
}