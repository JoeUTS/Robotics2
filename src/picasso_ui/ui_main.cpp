#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "picasso_ui.h"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PicassoUI>());
    rclcpp::shutdown();

    return 0;
}