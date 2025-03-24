#ifndef PICASSOEYES_H
#define PICASSOEYES_H

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

#include <librealsense2/rs.hpp>

#include "image_controller.h"

/*
Camera is realsense D435i

// Camera node: (Will need to launch this with launcher)
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
// alternative:
ros2 run realsense2_camera realsense2_camera_node
// from rosbag
ros2 launch realsense2_camera rs_launch.py rosbag_filename:="/full/path/to/rosbag/file" align_depth.enable:=true


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
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subCameraImage_; // Camera RGB image subscriber

  std::shared_ptr<imageController> imageController_ = NULL;

  void cameraImageCallback(const sensor_msgs::msg::Image::SharedPtr incomingMsg);
};

#endif // PICASSOEYES_H