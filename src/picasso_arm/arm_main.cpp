#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "picasso_arm.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PicassoArm>());
  rclcpp::shutdown();

  return 0;
}