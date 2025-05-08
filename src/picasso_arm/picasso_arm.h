#ifndef PICASSOARM_H
#define PICASSOARM_H

#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include "picasso_bot/srv/get_pose_array.hpp"

class PicassoArm : public rclcpp::Node {
public:
  PicassoArm(void);
  
  void moveToPose();  //  Declare moveToPose()
  void getCurrentPose(); // Declare setCurrentState()
  void getGoalPose(); // Declare moveRandomly()

private:

  rclcpp::TimerBase::SharedPtr timer_;  // Timer for periodic execution

  rclcpp::Client<picasso_bot::srv::GetPoseArray>::SharedPtr servNextContour_;
  bool prevContourExists_ = false;

  geometry_msgs::msg::PoseArray getNextContour(void);
  
};



#endif // PICASSOARM_H