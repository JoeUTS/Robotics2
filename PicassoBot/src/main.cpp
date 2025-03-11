#include <rclcpp/rclcpp.hpp>

// TO DO: Make program entry point.
#include "PicassoArm.h"
#include "PicassoEyes.h"
#include "PicassoUI.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PicassoEyes>());
    rclcpp::shutdown();
    return 0;
}