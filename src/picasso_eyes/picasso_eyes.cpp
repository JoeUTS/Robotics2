#include "picasso_eyes.h"

PicassoEyes::PicassoEyes(void) : Node("picaso_eyes") {
  // subscribers
  subCamera_ = this->create_subscription<realsense2_camera_msgs::msg::RGBD>(
    "/camera/camera/rgbd", 3, std::bind(&PicassoEyes::cameraReceiveCallback,this,std::placeholders::_1));

  // publishers
  pubVis_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/vis_toolpath", 3);
  pubCameraImage_ = this->create_publisher<sensor_msgs::msg::Image>("/processed_camera_image", 3);

  timer_ = this->create_wall_timer(
    timer_duration_, std::bind(&PicassoEyes::tempFunction, this));

}

void PicassoEyes::cameraReceiveCallback(const realsense2_camera_msgs::msg::RGBD::SharedPtr incomingMsg) {
  if (imageController_ == NULL) {
    imageController_ = std::make_shared<imageController>(incomingMsg);
    
  } else {
    sensor_msgs::msg::Image cameraImage = imageController_->updateCameraImage(incomingMsg);
    pubCameraImage_->publish(cameraImage);  // TO DO: make compressed version
  }
}

std::map<int, std::shared_ptr<Contour>> PicassoEyes::generateToolpath(cv::Mat &image, const float scale, const int blurPasses, const int blurKernalSize, const int colourSteps) {
  std::map<int, std::shared_ptr<Contour>> toolPaths;

  // Return on empty msg.
  if (image.empty()) {
    return toolPaths;
  }
  
  // Apply blur.
  for (int i = 0; i < blurPasses; i++) {
    imageController_->editImageBlurMedian(image, blurKernalSize);
    imageController_->editImageBlurGaussian(image, blurKernalSize);
  }
  
  // Reduce colour space.
  imageController_->editImageQuantize(image, colourSteps);

  // Get edges.
  cv::Mat edges = image.clone();
  imageController_->editImageGreyscale(edges);
  imageController_->detectEdges(edges, 0.1);

  // Generate contours.
  toolPaths = imageController_->getContours(edges, scale);

  return toolPaths;
}

void PicassoEyes::tempFunction(void) {
  cv::Mat imageRGB = imageController_->msg2Mat(imageController_->getStoredImage().rgb);

  if (imageRGB.empty()) {
    return;
  }

  RCLCPP_WARN(this->get_logger(), "Function running");

  // Function settings.
  // TO DO: Move elsewhere.
  double canvasWidth = 5;  // m.
  double scaleToolpath = canvasWidth / imageRGB.cols;  // Shrink to fit canvas.

  std_msgs::msg::ColorRGBA colourHead;
  colourHead.r = 0;
  colourHead.g = 1;
  colourHead.b = 0;
  colourHead.a = 1;

  std_msgs::msg::ColorRGBA colourTail;
  colourTail.r = 1;
  colourTail.g = 0;
  colourTail.b = 0;
  colourTail.a = 1;

  std_msgs::msg::ColorRGBA colourPath;
  colourTail.r = 0;
  colourTail.g = 0;
  colourTail.b = 1;
  colourTail.a = 1;

  geometry_msgs::msg::Vector3 scalePoint;
  scalePoint.x = 0.1;
  scalePoint.y = 0.1;
  scalePoint.z = 0.1;

  geometry_msgs::msg::Vector3 scaleLine;
  scaleLine.x = 1;
  scaleLine.y = 1;
  scaleLine.z = 1;


  // Localise people.
  // TO DO
  // - Return an array of detected people with location, size, bounding box ect.


  // Remove background
  // TO DO

  // Generate toolpaths34
  std::map<int, std::shared_ptr<Contour>> toolPaths = generateToolpath(imageRGB, scaleToolpath, 1, 3, 3);

  // Generate toolpaths visualization
  visualization_msgs::msg::MarkerArray markerArrayToolpath;
  unsigned int markerId = 0;

  for (auto &[key, contour] : toolPaths) {
    geometry_msgs::msg::Pose poseHead;
    poseHead.position = *contour->getHead();
    geometry_msgs::msg::Pose poseTail;
    poseTail.position = *contour->getTail();

    addMarkerPoint(markerArrayToolpath, markerId++, poseHead, scalePoint, colourHead);
    addMarkerPoint(markerArrayToolpath, markerId++, poseTail, scalePoint, colourHead);
    addMarkerPath(markerArrayToolpath, markerId++, contour->getPoints(), scaleLine, colourPath);
  }

  // Publish visualization
  if (markerArrayToolpath.markers.empty()) {
    return;
  }

  pubVis_->publish(markerArrayToolpath);
  
  
  // Perform TSP
  // TO DO

  // Publish pose array vs send via service

}

geometry_msgs::msg::Quaternion PicassoEyes::rpyToQuaternion(const double roll, const double pitch, const double yaw) {
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  geometry_msgs::msg::Quaternion q_msg;
  tf2::convert(q, q_msg);
  return q_msg;
}

void PicassoEyes::addMarkerPoint(visualization_msgs::msg::MarkerArray &markerArray, const unsigned int id, const geometry_msgs::msg::Pose &pose, const geometry_msgs::msg::Vector3 &scale, const std_msgs::msg::ColorRGBA &colour) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "camera_link";
  marker.header.stamp = this->get_clock()->now();
  marker.ns = "toolpath";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose  = pose;
  marker.scale = scale;
  marker.color = colour;

  markerArray.markers.push_back(marker);
}

void PicassoEyes::addMarkerPath(visualization_msgs::msg::MarkerArray &markerArray, const unsigned int id, const std::vector<geometry_msgs::msg::Point> &points, const geometry_msgs::msg::Vector3 &scale, const std_msgs::msg::ColorRGBA &colour) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "camera_link";
  marker.header.stamp = this->get_clock()->now();
  marker.ns = "toolpath";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;

  for (geometry_msgs::msg::Point point : points) {
    marker.points.push_back(point);
  }
  
  marker.scale = scale;
  marker.color = colour;

  markerArray.markers.push_back(marker);
}

cv::Mat PicassoEyes::getSketchPreview() {
  if (!imageController_) {
      RCLCPP_ERROR(this->get_logger(), "ImageController is not initialized.");
      return cv::Mat();
  }

  // Retrieve the stored image
  cv::Mat imageRGB = imageController_->msg2Mat(imageController_->getStoredImage().rgb);
  if (imageRGB.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No image available for sketch preview.");
      return cv::Mat();
  }

  // Generate toolpaths
  double canvasWidth = 5.0;  // Example canvas width in meters
  double scaleToolpath = canvasWidth / imageRGB.cols;  // Scale to fit canvas
  std::map<int, std::shared_ptr<Contour>> toolPaths = generateToolpath(imageRGB, scaleToolpath, 1, 3, 3);

  // Create a blank image to draw the sketch
  cv::Mat sketch = cv::Mat::zeros(imageRGB.size(), CV_8UC3);
  for (auto &[key, contour] : toolPaths) {
      // Convert ROS points to OpenCV points
      std::vector<cv::Point> cvPoints = convertToCvPoints(contour->getPoints());

      // Draw the contour on the sketch
      cv::polylines(sketch, cvPoints, false, cv::Scalar(255, 255, 255), 1);
  }

  return sketch;
}

std::vector<cv::Point> PicassoEyes::convertToCvPoints(const std::vector<geometry_msgs::msg::Point> &rosPoints) {
  std::vector<cv::Point> cvPoints;
  for (const auto &rosPoint : rosPoints) {
      cvPoints.emplace_back(cv::Point(static_cast<int>(rosPoint.x), static_cast<int>(rosPoint.y)));
  }
  return cvPoints;
}