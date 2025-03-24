#ifndef IMAGECONTROLLER_H
#define IMAGECONTROLLER_H

#include <mutex>

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <image_transport/image_transport.hpp>  // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <cv_bridge/cv_bridge.h>


class imageController {
public:
  imageController(const std::shared_ptr<rclcpp::Node> owningNode, const sensor_msgs::msg::Image::SharedPtr incomingMsg);

  /// @brief Updates the stored camera image message.
  /// @param incomingMsg 
  void updateCameraImage(const sensor_msgs::msg::Image::SharedPtr incomingMsg);

  void displayImage(sensor_msgs::msg::Image &imageMsg);

private:
  sensor_msgs::msg::Image msgCameraimage_;  // Holds last received image.

  std::mutex mutex_;
  std::shared_ptr<rclcpp::Node> owningNode_;
  
};

#endif // IMAGECONTROLLER_H




