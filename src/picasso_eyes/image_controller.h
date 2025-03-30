#ifndef IMAGECONTROLLER_H
#define IMAGECONTROLLER_H

#include <mutex>
#include <array>
#include <map>
#include <vector>
#include <string>
#include <thread>
#include <fstream>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <librealsense2/rs_advanced_mode.hpp>
#include <librealsense2/rsutil.h>
#include <realsense2_camera_msgs/msg/rgbd.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <image_transport/image_transport.hpp>  // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <cv_bridge/cv_bridge.h>

#include "Contour.h"

struct DetectedObject {
  int class_id;
  float confidence;
  cv::Rect box;
};

class imageController {
public:
  /// List of colours for drawing bounding boxes.
  const std::map<int, cv::Scalar> COLOURS_LIST = {
    {0, cv::Scalar(0, 0, 0)},
    {1, cv::Scalar(0, 0, 255)},
    {2, cv::Scalar(0, 255, 0)},
    {3, cv::Scalar(255, 0, 0)},
    {4, cv::Scalar(0, 255, 255)},
    {5, cv::Scalar(255, 0, 255)},
    {6, cv::Scalar(255, 255, 0)},
    {7, cv::Scalar(255, 255, 255)}
  };

  /// @brief Constructor for imageController. 
  imageController(const realsense2_camera_msgs::msg::RGBD::SharedPtr incomingMsg);

  /// @brief Updates the stored camera image message.
  /// @param incomingMsg 
  void updateCameraImage(const realsense2_camera_msgs::msg::RGBD::SharedPtr incomingMsg);

  /// @brief Gets the stored camera image message.
  /// @return realsense2_camera_msgs::msg::RGBD.
  realsense2_camera_msgs::msg::RGBD getStoredImage(void);
  
  /// @brief Gets the stored camera depth image message.
  /// @return cv::Mat.
  cv::Mat getDepthImage(void);

  /// @brief Gets the stored camera RGB image message.
  /// @return cv::Mat.
  cv::Mat getRGBImage(void);

  /// @brief Gets the stored camera RGB image message.
  /// @return sensor_msgs::msg::CameraInfo.
  sensor_msgs::msg::CameraInfo getRGBInfo(void);

  /// @brief Gets the stored camera depth image message.
  /// @return sensor_msgs::msg::CameraInfo.
  sensor_msgs::msg::CameraInfo getDepthInfo(void);

  /// @brief Gets the stored camera message header.
  /// @return std_msgs::msg::Header.
  std_msgs::msg::Header getHeader(void);

  /// @brief Blur and image via median blur using the given kernal size.
  /// @param image Image to blur.
  /// @param kernalSize Kernal size.
  void medianBlurImage(cv::Mat &image, const int kernalSize) {
    cv::medianBlur(image, image, kernalSize);
  }

  /// @brief Blur and image via gaussian blur using the given kernal size.
  /// @param image Image to blur.
  /// @param kernalSize Kernal size.
  void gaussianBlurImage(cv::Mat &image, const int kernalSize) {
    cv::GaussianBlur(image, image, cv::Size(kernalSize, kernalSize), 0);
  }

  /// @brief Converts an image to greyscale.
  /// @param image image to convert.
  void greyscaleImage(cv::Mat &image) {
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
  }

  /// @brief Thresholds an image with the given threshold
  /// @param image image to threshold.
  /// @param thresh threshold value.
  void thresholdImage(cv::Mat &image, const int thresh) {
    cv::threshold(image, image, thresh, 255, cv::THRESH_BINARY);
  }

  /// @brief Inverse thresholds an image with the given threshold
  /// @param image image to threshold.
  /// @param thresh threshold value.
  void thresholdInverseImage(cv::Mat &image, const int thresh) {
    cv::threshold(image, image, thresh, 255, cv::THRESH_BINARY_INV);
  }

  /// @brief Thresholds an image via adaptive gausian thresholding.
  /// @param image image to threshold.
  void adaptiveThresholdImage(cv::Mat &image, const int kernalSize) {
    cv::adaptiveThreshold(image, image, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, kernalSize, 2);
  }

  /// @brief Thresholds an image via Otsu's method thresholding.
  /// @param image image to threshold.
  void otsuThresholdImage(cv::Mat &image) {
    cv::threshold(image, image, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);
  }

  /// @brief Reduces an images colour space to the given number of steps using kmeans clustering. It is slow.
  /// @param image Image to edit.
  /// @param colourSteps Number of colours to reduce down to.
  void editImageQuantize(cv::Mat &image, const int colourSteps);

  /// @brief Gets the next colour from the list of colours.
  /// @return cv::Scalar.
  cv::Scalar getNextColour(void);

  /// @brief Split an image into RGB channels.
  /// @param image image to split.
  /// @return std::array<cv::Mat, 3> RGB colour channels.
  std::array<cv::Mat, 3> splitImage(cv::Mat &image);

  /// @brief Convert image msg to cv::Mat.
  /// @param imageMsg sensor_msgs::msg::Image
  /// @return cv::Mat.
  cv::Mat msg2Mat(sensor_msgs::msg::Image &imageMsg);

  /// @brief Detects edges in image using canny edge detection. \n
  ///        If multichannel input image, will convert to greyscale and blur.
  /// @param image Input image.
  /// @param thresh [0-1]. Edge detection wighting from lower (0) favoring fewer edges to upper (1) favoring more.
  void detectEdges(cv::Mat &image, float thresh = 0.33);

  /// @brief Find contours in an image. \n 
  ///        If multichannel input image, will convert to greyscale, blur and threshold.
  /// @param image Input image. If multichannel, will turn to greyscale and blur.
  /// @return List of contour objects.
  std::map<int, std::shared_ptr<Contour>> getContours(cv::Mat &image);

  /// @brief Calculate averate intensity of image. \n
  ///        If multichannel input image, will convert to greyscale and blur.
  /// @param image Input image.
  /// @return [0-255]. Average intensity.
  int calculateAverageIntensity(cv::Mat &image);

private:
  // system objects.
  std::mutex mutex_;  // Stored image mutex.

  int colourIndex_ = 0; // Index of next colour in colour list.
  
  // Threadding stuff.
  bool detectionRunning_ = false; // True when detection loop is active.
  

  // Messages.
  realsense2_camera_msgs::msg::RGBD lastCameraMsg_;  // Holds copy of last received image msg.

  // TO DO: test only
  void generateArt(void);

  void loadImageClassList(std::string path);
};

#endif // IMAGECONTROLLER_H