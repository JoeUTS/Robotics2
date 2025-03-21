#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "picasso_arm/picasso_arm.h"
#include "picasso_eyes/picasso_eyes.h"
#include "picasso_ui/picasso_ui.h"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("picasso_bot");  // Added by AI - Is this best practice?
    
    // Do we want to have each of these in their own node?
    //auto arm = std::make_shared<PicassoArm>(node); 
    auto eyes = std::make_shared<PicassoEyes>(node);
    //auto ui = std::make_shared<PicassoUI>(node);

    rclcpp::executors::MultiThreadedExecutor executor;  // Added by AI - what is this?
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
