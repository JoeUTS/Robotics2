#ifndef PICASSOARM_H
#define PICASSOARM_H

#include <rclcpp/rclcpp.hpp>

class PicassoArm : public rclcpp::Node {
public:
  PicassoArm(void);
  
  void moveToPose();  //  Declare moveToPose()
  void getCurrentPose(); // Declare setCurrentState()
  void getGoalPose(); // Declare moveRandomly()

private:

  rclcpp::TimerBase::SharedPtr timer_;  // Timer for periodic execution
  
};



#endif // PICASSOARM_H