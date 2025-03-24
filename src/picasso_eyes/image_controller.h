#ifndef IMAGECONTROLLER_H
#define IMAGECONTROLLER_H

#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"

#include "sensor_msgs/msg/image.hpp"

#include <librealsense2/rs_advanced_mode.hpp>
#include <librealsense2/rsutil.h>
#include <realsense2_camera_msgs/msg/rgbd.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <image_transport/image_transport.hpp>  // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <cv_bridge/cv_bridge.h>


class imageController {
public:
  imageController(const realsense2_camera_msgs::msg::RGBD::SharedPtr incomingMsg);

  /// @brief Updates the stored camera image message.
  /// @param incomingMsg 
  void updateCameraImage(const realsense2_camera_msgs::msg::RGBD::SharedPtr incomingMsg);

  // Display given image.
  void displayImage(cv::Mat &image);

private:
  bool detectionRunning_ = false;
  std::string testWin = "testWin";

  std::thread imageProcessThread_;

  // system objects.
  cv::Ptr<cv::LineSegmentDetector> lsd_;  // Line segment detector object.
  std::mutex mutex_;  // Mutex.

  // Messages.
  realsense2_camera_msgs::msg::RGBD msgCameraimage_;  // Holds copy of last received image msg.

  void generateArt(cv::Mat image);

  /// @brief Convert image msg to cv::Mat.
  /// @param imageMsg sensor_msgs::msg::Image
  /// @return cv::Mat.
  cv::Mat msg2Mat(sensor_msgs::msg::Image &imageMsg);

  /// @brief Detects edges in image using canny edge detection.
  /// @param image Input image. If multichannel, will turn to greyscale and blur.
  /// @param thresh Edge detection wighting from lower (0) favoring fewer edges to upper (1) favoring more.
  void edgeDetection(cv::Mat &image, float thresh = 0.33);

  /// @brief Find contours in an image.
  /// @param image Input image. If multichannel, will turn to greyscale and blur.
  /// @param contours Detected contours.
  /// @param hierarchy Hierachy of contours.
  void detectContour(cv::Mat &image, std::vector<std::vector<cv::Point>> &contours, std::vector<cv::Vec4i> &hierarchy);

  /// @brief Find averate intensity of greyscale image.
  /// @param image Input image. If multichannel, will turn to greyscale and blur.
  /// @return [0-255]. Average intensity.
  int findAverageIntensity(cv::Mat &image);
};

#endif // IMAGECONTROLLER_H