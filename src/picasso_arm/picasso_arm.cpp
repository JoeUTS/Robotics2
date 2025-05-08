#include "picasso_arm.h"
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using moveit::planning_interface::MoveGroupInterface;


PicassoArm::PicassoArm(void) : Node("picasso_arm") {
    RCLCPP_INFO(this->get_logger(), "PicassoArm node initialized.");

    timer_ = this->create_wall_timer(
        std::chrono::seconds(5), 
        std::bind(&PicassoArm::moveToPose, this) 
    );

    servNextContour_ = this->create_client<picasso_bot::srv::GetPoseArray>("/get_next_contour");
    
}

void PicassoArm::getGoalPose() {
    auto node_shared = shared_from_this();
    auto move_group_interface = MoveGroupInterface(node_shared, "ur_manipulator");

    // Get the current target pose (goal state)
    geometry_msgs::msg::PoseStamped currentGoalPoseStamped = move_group_interface.getPoseTarget();

    geometry_msgs::msg::Pose currentGoalPose_ = currentGoalPoseStamped.pose;

    RCLCPP_INFO(this->get_logger(), "Goal Pose: [x: %.3f, y: %.3f, z: %.3f]",
                currentGoalPose_ .position.x,
                currentGoalPose_ .position.y,
                currentGoalPose_ .position.z);
}


void PicassoArm::moveToPose() {

    auto node_shared = shared_from_this();
    auto move_group_interface = MoveGroupInterface(node_shared, "ur_manipulator");

    // Set a target Pose
   auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.position.x = 0.3;
    msg.position.y = 0.1;
    msg.position.z = 0.2;
    return msg;
   }();
   
   move_group_interface.setPoseTarget(target_pose);

   getGoalPose();

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
    

    //need a set current state function
    
    //hard code after tis    
    
}

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