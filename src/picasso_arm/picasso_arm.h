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
#include <tf2_eigen/tf2_eigen.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_srvs/srv/trigger.hpp>

#include "picasso_bot/srv/get_pose_array.hpp"
#include "picasso_bot/srv/get_total_lines.hpp"
#include "../common/ServiceCommon.h"

class MoveControl {
public:
  MoveControl(const std::shared_ptr<rclcpp::Node> owningNode, 
              const std::string moveGroupName, 
              const std::string endEffectorLink);

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> groupInterface_;

  
  /// @brief Plan trajectory to the input goal pose.
  /// @param startPose The start pose to set for the end effector.
  /// @param goalPose The goal pose to set for the end effector.
  /// @param plan [output] The planned trajectory.
  /// @return True if the planning was successful, false otherwise.
  bool planTrajectoryPoint(geometry_msgs::msg::Pose &startPose, 
                      geometry_msgs::msg::Pose &goalPose, 
                      moveit::planning_interface::MoveGroupInterface::Plan& plan);
  
  /// @brief Plan trajectory along the input waypoints.
  /// @param startPose The start pose to set for the end effector.
  /// @param goalPose The goal pose to set for the end effector.
  /// @param plan [output] The planned trajectory.
  /// @return True if the planning was successful, false otherwise.
  bool planTrajectoryPath(geometry_msgs::msg::Pose &startPose,
                        geometry_msgs::msg::PoseArray &waypoints, 
                        moveit::planning_interface::MoveGroupInterface::Plan& plan);

  /// @brief Move the end effector following the planned trajectory.
  /// @param plan The planned trajectory.
  /// @return True if the movement was successful, false otherwise.
  bool executePlan(moveit::planning_interface::MoveGroupInterface::Plan &plan);

private:
  const std::shared_ptr<rclcpp::Node> owningNode_;  // Node that owns this object
  const std::string moveGroupName_;                 // Name of the move group this controls
  const std::string endEffectorLink_;               // Name of the end effector link
  moveit::core::RobotModelConstPtr kinematicModel_; // Kinematic model of the robot

  /// @brief Convert a geometry_msgs::msg::Pose to a moveit::core::RobotState.
  /// @param pose The input pose to convert.
  /// @param robot_state The output robot state.
  /// @return True if the conversion was successful, false otherwise.
  bool poseToRobotState(const geometry_msgs::msg::Pose &pose, moveit::core::RobotState &robotState);
};

class PicassoArm : public rclcpp::Node {
public:
  PicassoArm(void);

private:
  const std::string moveGroupName_;
  const std::string endEffectorLink_;
  std::shared_ptr<MoveControl> moveController_; // Move controller for arm movements.
  rclcpp::TimerBase::SharedPtr timer_;  // Timer for periodic execution
  std::vector<geometry_msgs::msg::Point> target_points_; // List of target points to move through
  size_t current_target_index_ = 0;

  bool getNextContour(void);  // Get next contour from picasso eyes
  void drawImage(void);       // Draw image loop


  void getCurrentPose();  // Declare setCurrentState()
  void getGoalPose();     // Declare moveRandomly()
  void moveToNextPose();  // Declare moveToNextPose()
  void cartesianPath();   // Declare cartesianPath()
  void toolpath_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg); // Declare toolpath_callback()

  // Added by Joseph
  bool prevContourExists_;                      // Flag to check if first contour has been received
  bool contourResponce_;                        // Flag to indicate if the contour service has returned.
  bool totalLinesResponce_;                     // Flag to indicate if the total lines service has returned.
  geometry_msgs::msg::PoseArray toolPathMsg_;   // Received toolpath
  unsigned long totalLines_;                    // Total number of lines to draw
  std::string serviceLogName_;                  // Holds name of service for logging.

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr servConnectUR_;          // Connects to the UR robot
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr servStartDrawing_;       // Starts the drawing process
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr servStopDrawing_;        // Stops the drawing process
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr servHomePose_;           // Moves the arm to the home pose
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr servEStop;               // E-stop
  
  rclcpp::Client<picasso_bot::srv::GetPoseArray>::SharedPtr servNextContour_; // Gets the next contour to draw
  rclcpp::Client<picasso_bot::srv::GetTotalLines>::SharedPtr servTotalLines_; // gets the total number of contours in image.

  void serviceConnectUR(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);
  void serviceStartDrawing(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);
  void serviceStopDrawing(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);
  void serviceMoveToHome(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);
  void serviceEStop(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);

  void serviceNextContourRequest(void);
  void serviceNextContourRespose(rclcpp::Client<picasso_bot::srv::GetPoseArray>::SharedFuture future);
  void serviceTotalLinesRequest(void);
  void serviceTotalLinesRespose(rclcpp::Client<picasso_bot::srv::GetTotalLines>::SharedFuture future);
};

#endif // PICASSO_ARM_H
