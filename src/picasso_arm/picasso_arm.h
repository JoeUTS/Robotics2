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
#include <std_srvs/srv/trigger.hpp>

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
  std::vector<geometry_msgs::msg::Point> target_points_; // List of target points to move through
  size_t current_target_index_ = 0;

  // Added by Joseph
  bool prevContourExists_ = false;              // Flag to check if first contour has been received
  geometry_msgs::msg::PoseArray toolPathMsg_;   // Received toolpath
  unsigned int toolPathIndex_ = 0;              // Index of the current toolpath

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr servStartDrawing_;       // Starts the drawing process
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr servStopDrawing_;        // Stops the drawing process
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr servHomePose_;           // Moves the arm to the home pose
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr servEStop;               // E-stop
  
  rclcpp::Client<picasso_bot::srv::GetPoseArray>::SharedPtr servNextContour_; // Gets the next contour to draw

  void serviceStartDrawing(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);
  void serviceStopDrawing(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);
  void serviceMoveToHome(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);
  void serviceEStop(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);

  geometry_msgs::msg::PoseArray getNextContour(void);
  void serviceNextContourRequest(void);
  void serviceNextContourRespose(rclcpp::Client<picasso_bot::srv::GetPoseArray>::SharedFuture future);
};



#endif // PICASSO_ARM_H
