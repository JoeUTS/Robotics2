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

const std::array<cv::Scalar, 8> COLOURS = {cv::Scalar(0, 0, 0),
                                          cv::Scalar(0, 0, 255),
                                          cv::Scalar(0, 255, 0),
                                          cv::Scalar(255, 0, 0),
                                          cv::Scalar(0, 255, 255),
                                          cv::Scalar(255, 0, 255),
                                          cv::Scalar(255, 255, 0),
                                          cv::Scalar(255, 255, 255)
};

// TO DO:
// - Add function to set toolpaths.
// - Add function to clear toolpaths.
// - Add function to get next toolpath. (This will later be used with traveling salesman problem.)

class imageController {
public:
  imageController(const realsense2_camera_msgs::msg::RGBD::SharedPtr incomingMsg);

  /// @brief Updates the stored camera image message.
  /// @param incomingMsg 
  void updateCameraImage(const realsense2_camera_msgs::msg::RGBD::SharedPtr incomingMsg);

private:
  // system objects.
  std::mutex mutex_;  // Mutex.
  bool detectionRunning_ = false; // True when detection loop is active.
  std::thread imageProcessThread_;

  std::vector<std::shared_ptr<Contour>> toolPaths_;
  
  // Object Detection.
  std::vector<std::string> classList_; // Image classifier classes.

  // Messages.
  realsense2_camera_msgs::msg::RGBD msgCameraimage_;  // Holds copy of last received image msg.

  void generateArt(void);

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
  void detectContour(cv::Mat image, std::vector<std::vector<cv::Point>> &contours, std::vector<cv::Vec4i> &hierarchy);

  /// @brief Find averate intensity of greyscale image.
  /// @param image Input image. If multichannel, will turn to greyscale and blur.
  /// @return [0-255]. Average intensity.
  int findAverageIntensity(cv::Mat &image);

  void loadImageClassList(std::string path);

  void setColourList(void);
};

#endif // IMAGECONTROLLER_H