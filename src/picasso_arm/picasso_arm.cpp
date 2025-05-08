#include "picasso_arm.h"
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include "picasso_eyes.h"

using moveit::planning_interface::MoveGroupInterface;

PicassoArm::PicassoArm(void) : Node("picasso_arm") {
    RCLCPP_INFO(this->get_logger(), "PicassoArm node initialized.");

    // eyes_ = std::make_shared<PicassoEyes>();

    // toolpath_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    //     "/eyes_toolpath", 10,  // <-- replace with actual topic name if different
    //     std::bind(&PicassoArm::toolpath_callback, this, std::placeholders::_1)
    // );

    timer_ = this->create_wall_timer(
        std::chrono::seconds(5), 
        std::bind(&PicassoArm::moveToPose, this) 
    );

    servNextContour_ = this->create_client<picasso_bot::srv::GetPoseArray>("/get_next_contour");
    
}

// void PicassoArm::moveToNextPose() {

//     if (current_target_index_ >= target_points_.size()) {
//         RCLCPP_INFO(this->get_logger(), "All target points reached.");
//         timer_->cancel();
//         return;
//     }
//     auto node_shared = shared_from_this();
//     auto move_group_interface = MoveGroupInterface(node_shared, "ur_manipulator");
   
//     move_group_interface.setEndEffectorLink("tool0");

//     geometry_msgs::msg::Pose target_pose;
//     target_pose.position = target_points_[current_target_index_];

//     move_group_interface.setPoseTarget(target_pose);
   
//     auto const [success, plan] = [&move_group_interface]{
//         moveit::planning_interface::MoveGroupInterface::Plan msg;
//         auto const ok = static_cast<bool>(move_group_interface.plan(msg));
//         return std::make_pair(ok, msg);
//     }();

//     if(success) {
//         move_group_interface.execute(plan);
//         RCLCPP_INFO(this->get_logger(), "Moved to target point %zu", current_target_index_);
//     } else {
//         RCLCPP_ERROR(this->get_logger(), "Planning failed for target point %zu!", current_target_index_);
//     }
// }


void PicassoArm::moveToPose() {

    auto node_shared = shared_from_this();
    auto move_group_interface = MoveGroupInterface(node_shared, "ur_manipulator");

    move_group_interface.setEndEffectorLink("tool0");

    // Set a target Pose
   auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.position.x = 0.5;
    msg.position.y = 0.5;
    msg.position.z = 0.5;
    return msg;
   }();
   
    move_group_interface.setPoseTarget(target_pose);
 
    // Create a plan to that target pose
    auto const [success, plan] = [&move_group_interface]{
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();


    // Execute the plan
    if(success) {
     move_group_interface.execute(plan);
     RCLCPP_WARN(this->get_logger(), "Completed function");               
     timer_->cancel();

     
    } else {
     RCLCPP_ERROR(this->get_logger(), "Planning failed!");
    }
}
// void PicassoArm::toolpath_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
//     RCLCPP_INFO(this->get_logger(), "Received toolpath with %zu poses.", msg->poses.size());

//     target_points_.clear();
//     current_target_index_ = 0;

//     for (const auto& pose : msg->poses) {
//         target_points_.push_back(pose.position);
//     }

//     if (!target_points_.empty()) {
//         RCLCPP_INFO(this->get_logger(), "Toolpath loaded. Starting movement...");
//         timer_->reset();  // Restart timer to begin moving
//     } else {
//         RCLCPP_WARN(this->get_logger(), "Received empty toolpath.");
//     }
// }

/*
How to use below service:
// Service already set up.

geometry_msgs::msg::PoseArray contourMsg = getNextContour();   // Request next contour

if (contourMsg.poses.size() == 0) {
    // No contour to receive and/or end of contours.
}

// contourMsg will contain a header and a vector of geometry_msgs::msg::Pose

// to access poses:
geometry_msgs::msg::Pose goalPose = contourMsg.poses.at(X); // where X is the index of the goal pose

// if you wanted a function that itterates though the list:
while (contourMsg.poses.size() > index) {
    geometry_msgs::msg::Pose goalPose = contourMsg.poses.at(index);
    index++;
}

*/

geometry_msgs::msg::PoseArray PicassoArm::getNextContour(void) {
    auto messagePeriod = std::chrono::milliseconds(1000);
    std::chrono::time_point<std::chrono::system_clock> lastMsg;
    
    // Wait for service
    while (!servNextContour_->wait_for_service(std::chrono::milliseconds(200))) {
        // Prevent spaming messages
        std::chrono::duration<double> duration = std::chrono::system_clock::now() - lastMsg;
        std::chrono::milliseconds timeSinceLastMsg = std::chrono::duration_cast<std::chrono::milliseconds>(duration);
        
        if (timeSinceLastMsg >= messagePeriod) {
            lastMsg = std::chrono::system_clock::now();
            RCLCPP_INFO_STREAM(this->get_logger(), "waiting for service 'get_next_contour' to connect");
        }
    }

    auto request = std::make_shared<picasso_bot::srv::GetPoseArray::Request>();
    auto result = servNextContour_->async_send_request(request);
    
    geometry_msgs::msg::PoseArray contour = geometry_msgs::msg::PoseArray();

    // Await responce
    if (rclcpp::spin_until_future_complete(this->shared_from_this(), result) == rclcpp::FutureReturnCode::SUCCESS) {
        bool success = result.get()->success;
        
        if (success) {
            RCLCPP_INFO_STREAM(this->get_logger(), "Contour received.");
            success = true;
            prevContourExists_ = true;
            contour = result.get()->poses;

        } else if (prevContourExists_ == false) {
            RCLCPP_INFO_STREAM(this->get_logger(), "Drawing complete.");

        } else {
            RCLCPP_WARN_STREAM(this->get_logger(), "Failed to get next contour.");
        }

    } else {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service 'get_next_contour'");
    }

    return contour;
}