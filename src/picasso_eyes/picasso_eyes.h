#ifndef PICASSOEYES_H
#define PICASSOEYES_H

#include <string>
#include <thread>
#include <map>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <realsense2_camera_msgs/msg/rgbd.hpp>
#include <opencv2/opencv.hpp>

#include "image_controller.h"
#include "Contour.h"
#include "salesman_solver.h"

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
  cv::Mat getSketchPreview();
private:
  // Topics
  rclcpp::Subscription<realsense2_camera_msgs::msg::RGBD>::SharedPtr subCamera_;  // Camera RGB image subscriber.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubVis_;     // Visualization markers publisher.
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pubCameraImage_;          // Camera image publisher.

  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr servCameraToggleRepub_;      // Toggle republishing camera feed to UI.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr servCaptureImage_;           // Capture image for sketch.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr servPreviewSketch_;          // Generate sketch preview from captured image.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr servDiscardImage_;           // Discard captured image/sketch.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr servDrawSketch_;             // Start drawing sequence. Setting to false will cease drawing.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr servNextContour_;            // Change published contour. Will publish empty poseArray when complete.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr serviceShutdown_;            // Shutdown node.

  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::milliseconds timer_duration_{1000};
  
  std::shared_ptr<imageController> imageController_ = NULL;
  std::shared_ptr<SalesmanSolver> salesmanSolver_ = NULL;

  bool cameraFeedEnabled_ = false;
  bool imageCaptured_ = false;
  int pubCompressQuality_ = 80;
  sensor_msgs::msg::Image capturedImageMsg_;      // Holds copy of last captured image msg.
  cv::Mat capturedImage_;                         // Holds copy of last captured image.
  std::map<int, std::shared_ptr<Contour>> toolPaths_;
  std::thread imageProcessThread_;
  std::vector<std::pair<int, bool>> contourOrder_;

  geometry_msgs::msg::PoseArray outputPoseArray_; // Holds copy of last generated pose array msg.
  
  
  /// @brief Updates ImageController with new camera image. If ImageController object not made, creates.
  /// @param incomingMsg sensor_msgs::msg::Image
  void callbackCameraReceive(const realsense2_camera_msgs::msg::RGBD::SharedPtr incomingMsg);

  /// @brief Service callback for toggling camera feed.
  void serviceToggleCameraFeed(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);

  /// @brief Service callback for capturing image for sketch.
  void serviceCaptureImage(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);

  /// @brief Service callback for previewing sketch from captured image.
  void servicePreviewSketch(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);

  /// @brief Service callback for discarding captured image.
  void serviceDiscardImage(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);

  /// @brief Service callback for drawing sketch.
  void serviceDrawSketch(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);

  /// @brief Service callback for changing the published contour. - SHOULD THIS BE A CUSTOM SERVICE TO RESPOND WITH A POSEARRAY?
  void serviceNextContour(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);

  /// @brief Service callback for shutting down node.
  void serviceShutdown(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);

  /// @brief Compress image to specified quality.
  /// @param imageMsg Image to compress.
  /// @param quality Quality to compress to.
  sensor_msgs::msg::Image compressImage(sensor_msgs::msg::Image &imageMsg, const int quality);

  cv::Mat captureImage(void);

  /// @brief Generate toolpath from provided image.
  /// @param image image to generate toolpath from. \n
  /// @param blurPasses Number of blur passes to apply. \n
  ///                   Note: This is applied prior to colour quantisation. \n
  ///                   Note: Applies both a gaussian and median blur per pass.
  /// @param blurKernalSize size of blur kernal.
  /// @param colourSteps Number of colours to reduce to.
  /// @return Vector of contours.
  std::map<int, std::shared_ptr<Contour>> PicassoEyes::generateToolpath(cv::Mat image, const bool visualise = false);

  /// @brief Generate sketch from input image.
  cv::Mat generateSketch(cv::Mat image, const int blurPasses = 1, const int blurKernalSize = 3, const int colourSteps = 3, const bool showEdges = false);


  /// @brief Temporary function for testing.
  void tempFunction(void);

  geometry_msgs::msg::Quaternion rpyToQuaternion(const double roll, const double pitch, const double yaw);

  void addMarkerPoint(visualization_msgs::msg::MarkerArray &markerArray, const unsigned int id, const geometry_msgs::msg::Pose &pose, const geometry_msgs::msg::Vector3 &scale, const std_msgs::msg::ColorRGBA &colour);

  void addMarkerPath(visualization_msgs::msg::MarkerArray &markerArray, const unsigned int id, const std::vector<geometry_msgs::msg::Point> &points, const geometry_msgs::msg::Vector3 &scale, const std_msgs::msg::ColorRGBA &colour);

  std::vector<cv::Point> convertToCvPoints(const std::vector<geometry_msgs::msg::Point> &rosPoints);
};

#endif // PICASSOEYES_H