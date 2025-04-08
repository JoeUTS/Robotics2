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