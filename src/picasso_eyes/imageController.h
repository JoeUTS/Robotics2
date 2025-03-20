#ifndef IMAGECONTROLLER_H
#define IMAGECONTROLLER_H

#include <mutex>

#include <sensor_msgs/msg/image.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>


class imageController {
public:
  imageController(const sensor_msgs::msg::Image::SharedPtr incomingMsg);

  /// @brief Updates the stored camera image message.
  /// @param incomingMsg 
  void updateCameraImage(const sensor_msgs::msg::Image::SharedPtr incomingMsg);

private:
  sensor_msgs::msg::Image msgCameraimage_;  // Holds last received image.

  std::mutex mutex_;
  
};

#endif // IMAGECONTROLLER_H




