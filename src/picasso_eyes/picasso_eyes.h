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
#include "VisulisationSettings.h"
#include "picasso_bot/srv/get_pose_array.hpp"
#include "picasso_bot/srv/get_image.hpp"

/*
// Camera node: (Will need to launch this with launcher)
ros2 launch realsense2_camera rs_launch.py enable_rgbd:=true rgb_camera.color_profile:=640x480x30 depth_module.depth_profile:=640x480x30 enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true
*/

class PicassoEyes : public rclcpp::Node {
public:
  PicassoEyes(void);
  
private:
  // settings
  int pubCompressQuality_ = 80;

  // Topics
  rclcpp::Subscription<realsense2_camera_msgs::msg::RGBD>::SharedPtr subCamera_;  // Camera RGB image subscriber.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubVis_;     // Visualization markers publisher.
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pubCameraImage_;          // Camera image publisher.

  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr servCameraToggleRepub_;      // Toggle republishing camera feed to UI.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr servCaptureImage_;           // Capture image for sketch.
  rclcpp::Service<picasso_bot::srv::GetImage>::SharedPtr servPreviewSketch_;          // Generate sketch preview from captured image.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr servDiscardImage_;           // Discard captured image/sketch.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr servGenerateToolpath_;       // Generate draw order.
  rclcpp::Service<picasso_bot::srv::GetPoseArray>::SharedPtr servNextContour_; // Send next contour to draw.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr serviceShutdown_;            // Shutdown node.
  
  // Class objects
  std::shared_ptr<imageController> imageController_ = NULL;
  std::shared_ptr<SalesmanSolver> salesmanSolver_ = NULL;

  // --- Runtime variables ---
  // Image capture
  bool cameraFeedEnabled_ = false;
  bool imageCaptured_ = false;
  sensor_msgs::msg::Image capturedImageMsg_;      // Holds copy of last captured image msg.
  cv::Mat capturedImage_;                         // Holds copy of last captured image.

  // Mask generation
  std::unique_ptr<std::thread> maskThread_ = NULL;
  bool maskGenerationActive_ = false; // True when generation function is active.
  bool maskReady_ = false;            // True when mask is ready.
  cv::Mat mask_;                      // Generated mask.

  // Toolpath generation
  std::map<int, std::shared_ptr<Contour>> toolPaths_;
  std::vector<std::pair<int, bool>> contourOrder_;
  int contourOrderIndex_ = 0;
  
  
  /// @brief Updates ImageController with new camera image. If ImageController object not made, creates.
  /// @param incomingMsg sensor_msgs::msg::Image
  void callbackCameraReceive(const realsense2_camera_msgs::msg::RGBD::SharedPtr incomingMsg);

  /// @brief Service callback for toggling camera feed.
  void serviceToggleCameraFeed(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);

  /// @brief Service callback for capturing image for sketch.
  void serviceCaptureImage(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);

  /// @brief Service callback for previewing sketch from captured image.
  void servicePreviewSketch(const picasso_bot::srv::GetImage::Request::SharedPtr request, picasso_bot::srv::GetImage::Response::SharedPtr response);

  /// @brief Service callback for discarding captured image.
  void serviceDiscardImage(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);

  /// @brief Service callback for drawing sketch.
  void serviceGenerateToolpath(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);

  /// @brief Service callback for changing the published contour. - SHOULD THIS BE A CUSTOM SERVICE TO RESPOND WITH A POSEARRAY?
  void serviceNextContour(const picasso_bot::srv::GetPoseArray::Request::SharedPtr request, picasso_bot::srv::GetPoseArray::Response::SharedPtr response);

  /// @brief Service callback for shutting down node.
  void serviceShutdown(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);

  /// @brief Compress image to specified quality.
  /// @param imageMsg Image to compress.
  /// @param quality Quality to compress to.
  sensor_msgs::msg::Image compressImage(sensor_msgs::msg::Image &imageMsg, const int quality);

  /// @brief Generate toolpath from provided image.
  /// @param image image to generate toolpath from. \n
  /// @param blurPasses Number of blur passes to apply. \n
  ///                   Note: This is applied prior to colour quantisation. \n
  ///                   Note: Applies both a gaussian and median blur per pass.
  /// @param blurKernalSize size of blur kernal.
  /// @param colourSteps Number of colours to reduce to.
  /// @return Vector of contours.
  std::map<int, std::shared_ptr<Contour>> generateToolpath(cv::Mat &image, const bool visualise = false);

  /// @brief Generate sketch from input image.
  cv::Mat generateSketch(cv::Mat &image, const int blurPasses = 1, const int blurKernalSize = 3, const int colourSteps = 3, const bool showEdges = false);

  /// @brief Temporary function for testing.
  void tempFunction(void);

  geometry_msgs::msg::Quaternion rpyToQuaternion(const double roll, const double pitch, const double yaw);

  void addMarkerPoint(visualization_msgs::msg::MarkerArray &markerArray, const unsigned int id, const geometry_msgs::msg::Pose &pose, const geometry_msgs::msg::Vector3 &scale, const std_msgs::msg::ColorRGBA &colour);

  void addMarkerPath(visualization_msgs::msg::MarkerArray &markerArray, const unsigned int id, const std::vector<geometry_msgs::msg::Point> &points, const geometry_msgs::msg::Vector3 &scale, const std_msgs::msg::ColorRGBA &colour);

  std::vector<cv::Point> convertToCvPoints(const std::vector<geometry_msgs::msg::Point> &rosPoints);

  /// @brief Starts mask generation in a separate thread. Only on instance can run at a time.
  void generateMask(cv::Mat &image);
};

#endif // PICASSOEYES_H