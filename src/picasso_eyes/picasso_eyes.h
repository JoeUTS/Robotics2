#ifndef PICASSOEYES_H
#define PICASSOEYES_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"

#include "std_msgs/msg/header.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <realsense2_camera_msgs/msg/rgbd.hpp>
#include <opencv2/opencv.hpp>

#include <string>
#include <thread>
#include <map>
#include <chrono>

#include "image_controller.h"
#include "Contour.h"

/*
Camera is realsense D435i

// Camera node: (Will need to launch this with launcher)
ros2 launch realsense2_camera rs_launch.py enable_rgbd:=true rgb_camera.color_profile:=640x480x30 depth_module.depth_profile:=640x480x30 enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true

// from rosbag
ros2 launch realsense2_camera rs_launch.py rosbag_filename:="/full/path/to/rosbag/file" align_depth.enable:=true

// arm
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.1.102 launch_rviz:=false initial_joint_controller:=joint_trajectory_controller use_fake_hardware:=true

ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true use_fake_hardware:=true

// Default topic list:
https://dev.intelrealsense.com/docs/ros2-wrapper
> ros2 node list
/camera/camera

> ros2 topic list
/camera/camera/color/camera_info
/camera/camera/color/image_raw
/camera/camera/color/metadata
/camera/camera/depth/camera_info
/camera/camera/depth/image_rect_raw
/camera/camera/depth/metadata
/camera/camera/extrinsics/depth_to_color
/camera/camera/imu

> ros2 service list
/camera/camera/device_info

// Useful links:
https://docs.ros.org/en/humble/p/librealsense2/doc/d435i.html
https://github.com/IntelRealSense/librealsense/tree/development
*/

class PicassoEyes : public rclcpp::Node {
public:
  PicassoEyes(void);

private:
  bool generationRunning_ = false;
  std::thread imageProcessThread_;

  rclcpp::Subscription<realsense2_camera_msgs::msg::RGBD>::SharedPtr subCamera_;  // Camera RGB image subscriber.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubVis_;     // Visualization markers publisher.
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pubCameraImage_;          // Camera image publisher.
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::milliseconds timer_duration_{1000};
  
  std::shared_ptr<imageController> imageController_ = NULL;

  geometry_msgs::msg::PoseArray outputPoseArray_; // Holds copy of last generated pose array msg.

  void cameraReceiveCallback(const realsense2_camera_msgs::msg::RGBD::SharedPtr incomingMsg);

  /// @brief Generate toolpath from provided image.
  /// @param image image to generate toolpath from.
  /// @param blurPasses Number of blur passes to apply. \n
  ///                   Note: This is applied prior to colour quantisation. \n
  ///                   Note: Applies both a gaussian and median blur per pass.
  /// @param blurKernalSize size of blur kernal.
  /// @param colourSteps Number of colours to reduce to.
  /// @return Vector of contours.
  std::map<int, std::shared_ptr<Contour>> generateToolpath(cv::Mat &image, const float scale, const int blurPasses, const int blurKernalSize, const int colourSteps);

  /// @brief Temporary function for testing.
  void tempFunction(void);

  geometry_msgs::msg::Quaternion rpyToQuaternion(const double roll, const double pitch, const double yaw);

  void addMarkerPoint(visualization_msgs::msg::MarkerArray &markerArray, const unsigned int id, const geometry_msgs::msg::Pose &pose, const geometry_msgs::msg::Vector3 &scale, const std_msgs::msg::ColorRGBA &colour);

  void addMarkerPath(visualization_msgs::msg::MarkerArray &markerArray, const unsigned int id, const std::vector<geometry_msgs::msg::Point> &points, const geometry_msgs::msg::Vector3 &scale, const std_msgs::msg::ColorRGBA &colour);
};

#endif // PICASSOEYES_H