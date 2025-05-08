#ifndef PICASSOARM_H
#define PICASSOARM_H

#include <rclcpp/rclcpp.hpp>

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
  std::shared_ptr<PicassoEyes> eyes_;                // Pointer to the Eyes system
  std::vector<geometry_msgs::msg::Point> target_points_; // List of target points to move through
  size_t current_target_index_ = 0;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr toolpath_subscriber_;

  
};



#endif // PICASSOARM_H