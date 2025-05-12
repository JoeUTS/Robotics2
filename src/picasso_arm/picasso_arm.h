#ifndef PICASSO_ARM_H
#define PICASSO_ARM_H

#include <chrono>
#include <memory>
#include <functional>
#include <string>
#include <iostream> // Should be using RCLCPP_INFO(this->get_logger(), "") or RCLCPP_INFO_STREAM(this->get_logger(), "") instead
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include "picasso_bot/srv/get_pose_array.hpp"
#include "../common/ServiceCommon.h"

class PicassoArm : public rclcpp::Node {
public:
  PicassoArm(void);
  
  void moveToPose();  //  Declare moveToPose()
  void getCurrentPose(); // Declare setCurrentState()
  void getGoalPose(); // Declare moveRandomly()
  void moveToNextPose(); // Declare moveToNextPose()
  void cartesianPath(); // Declare cartesianPath()
  void toolpath_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg); // Declare toolpath_callback()


private:

  rclcpp::TimerBase::SharedPtr timer_;  // Timer for periodic execution
  //std::shared_ptr<PicassoEyes> eyes_;                // Pointer to the Eyes system
  std::vector<geometry_msgs::msg::Point> target_points_; // List of target points to move through
  size_t current_target_index_ = 0;
  //rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr toolpath_subscriber_; // not needed


  rclcpp::Client<picasso_bot::srv::GetPoseArray>::SharedPtr servNextContour_;

  bool prevContourExists_ = false;
  geometry_msgs::msg::PoseArray toolPathMsg_;
  unsigned int toolPathIndex_ = 0;
  geometry_msgs::msg::PoseArray getNextContour(void);
  void serviceNextContourRequest(void);
  void serviceNextContourRespose(rclcpp::Client<picasso_bot::srv::GetPoseArray>::SharedFuture future);
};



#endif // PICASSO_ARM_H
