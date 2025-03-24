#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "picasso_eyes.h"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PicassoEyes>());
    rclcpp::shutdown();

    return 0;
}