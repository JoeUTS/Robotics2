#ifndef PICASSO_ARM_H
#define PICASSO_ARM_H

#include <chrono>
#include <memory>
#include <functional>
#include <string>
#include <iostream>
#include <vector>
#include <type_traits>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen/tf2_eigen.hpp>
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
  
  /// @brief Move to predefined home pose.
  void moveToHome(void);
  
  /// @brief Plan trajectory to the input goal pose.
  /// @param goalPose The goal pose to set for the end effector.
  /// @param plan [output] The planned trajectory.
  /// @return True if the planning was successful, false otherwise.
  bool planTrajectoryPoint(geometry_msgs::msg::Pose &goalPose, 
                      moveit::planning_interface::MoveGroupInterface::Plan& plan);
  
  /// @brief Plan trajectory along the input waypoints.
  /// @param goalPose The goal pose to set for the end effector.
  /// @param plan [output] The planned trajectory.
  /// @return True if the planning was successful, false otherwise.
  bool planTrajectoryPath(geometry_msgs::msg::PoseArray &waypoints, 
                        moveit::planning_interface::MoveGroupInterface::Plan& plan);

  /// @brief Move the end effector following the planned trajectory.
  /// @param plan The planned trajectory.
  /// @return True if the movement was successful, false otherwise.
  bool executePlan(moveit::planning_interface::MoveGroupInterface::Plan &plan);

private:
  const std::shared_ptr<rclcpp::Node> owningNode_;  // Node that owns this object
  const std::string moveGroupName_;                 // Name of the move group this controls
  const std::string endEffectorLink_;               // Name of the end effector link
};

class PicassoArm : public rclcpp::Node {
public:
  PicassoArm(void);

private:
  // My dude, you dont need to sign you own code. I have been putting my name next to stuff to communicate with you haha.


  const std::string moveGroupName_;
  const std::string endEffectorLink_;
  std::shared_ptr<MoveControl> moveController_; // Move controller for arm movements.

  bool getNextContour(void);  // Get next contour from picasso eyes
  void drawImage(void);       // Draw image loop

  void getCurrentPose();
  void getGoalPose(); 
  void cartesianPath();
  void toolpath_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg); 

  void pauseDrawing();                          // Pause the drawing process
  void resumeDrawing();                         // Resume the drawing process
  bool drawing_active_;                         // Flag to indicate if the drawing process is active
  bool drawing_paused_;                         // Flag to indicate if the drawing process is paused
 
  // Added by Joseph
  bool prevContourExists_;                      // Flag to check if first contour has been received
  bool contourResponce_;                        // Flag to indicate if the contour service has returned.
  bool totalLinesResponce_;                     // Flag to indicate if the total lines service has returned.
  geometry_msgs::msg::PoseArray toolPathMsg_;   // Received toolpath
  unsigned int totalLines_;                     // Total number of lines to draw
  unsigned int currentLine_;                    // Current line being drawn
  std::string serviceLogName_;                  // Holds name of service for logging.

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr servConnectUR_;          // Connects to the UR robot
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr servStartDrawing_;       // Starts the drawing process
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr servStopDrawing_;        // Stops the drawing process
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr servHomePose_;           // Moves the arm to the home pose
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr servEStop;               // E-stop
  
  rclcpp::Client<picasso_bot::srv::GetPoseArray>::SharedPtr servNextContour_; // Gets the next contour to draw
  
  void serviceConnectUR(const std_srvs::srv::Trigger::Request::SharedPtr request, 
                        const std_srvs::srv::Trigger::Response::SharedPtr response);

  void serviceStartDrawing(const std_srvs::srv::Trigger::Request::SharedPtr request, 
                        const std_srvs::srv::Trigger::Response::SharedPtr response);

  void serviceStopDrawing(const std_srvs::srv::Trigger::Request::SharedPtr request, 
                        const std_srvs::srv::Trigger::Response::SharedPtr response);

  void serviceMoveToHome(const std_srvs::srv::Trigger::Request::SharedPtr request, 
                        const std_srvs::srv::Trigger::Response::SharedPtr response);

  void serviceEStop(const std_srvs::srv::Trigger::Request::SharedPtr request, 
                        const std_srvs::srv::Trigger::Response::SharedPtr response);

  void serviceNextContourRequest(void);
};

#endif // PICASSO_ARM_H
