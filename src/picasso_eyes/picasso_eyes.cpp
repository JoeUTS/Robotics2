#include "picasso_eyes.h"

PicassoEyes::PicassoEyes(void) : Node("picaso_eyes") {
  // subscribers
  subCameraMsg_ = this->create_subscription<realsense2_camera_msgs::msg::RGBD>(
    "/camera/camera/rgbd", 1000, std::bind(&PicassoEyes::cameraReceiveCallback,this,std::placeholders::_1));

}

void PicassoEyes::cameraReceiveCallback(const realsense2_camera_msgs::msg::RGBD::SharedPtr incomingMsg) {
  if (imageController_ == NULL) {
    imageController_ = std::make_shared<imageController>(incomingMsg);
  } else {
    imageController_->updateCameraImage(incomingMsg);
  }

  // Compress image and republish for UI.
  // TO DO

  if (!generationRunning_) {
    generationRunning_ = true;
    imageProcessThread_ = std::thread(&PicassoEyes::tempFunction, this);
    imageProcessThread_.detach();
  }
}

std::map<int, std::shared_ptr<Contour>> PicassoEyes::generateToolpath(cv::Mat &image, const int blurPasses, const int blurKernalSize, const int colourSteps) {
  std::map<int, std::shared_ptr<Contour>> toolPaths;

  // Return on empty msg.
  if (image.empty()) {
    return toolPaths;
  }
  
  // Apply blur.
  for (int i = 0; i < blurPasses; i++) {
    imageController_->medianBlurImage(image, blurKernalSize);
    imageController_->gaussianBlurImage(image, blurKernalSize);
  }
  
  // Reduce colour space.
  imageController_->editImageQuantize(image, colourSteps);

  // Get edges.
  cv::Mat edges = image.clone();
  imageController_->greyscaleImage(edges);
  imageController_->detectEdges(edges, 0.1);

  // Generate contours.
  toolPaths = imageController_->getContours(edges);

  return toolPaths;
}

void PicassoEyes::tempFunction(void) {
  cv::Mat imageRGB = imageController_->getRGBImage();

  if (imageRGB.empty()) {
    generationRunning_ = false;
    return;
  }

  // Localise people.
  // TO DO
  // - Return an array of detected people with location, size, bounding box ect.


  // Remove background
  // TO DO

  // Generate toolpaths
  std::map<int, std::shared_ptr<Contour>> toolPaths = generateToolpath(imageRGB, 1, 3, 3);

  // Generate toolpaths visualization
  // TO DO
  visualization_msgs::msg::MarkerArray markerArrayToolpath;
  unsigned int markerId = 0;
  for (std::shared_ptr<Contour> &contour : toolPaths) {
    visualization_msgs::msg::Marker tempMarker;
    tempMarker.header.frame_id = "camera_link";
    tempMarker.header.stamp = this->get_clock()->now();
    tempMarker.ns = "toolpath";
    tempMarker.action = 0;  // add/modify

    // Start marker.
    tempMarker.id = markerId++;
    tempMarker.type = 0;  // Arrow
    tempMarker.pose.position = contour->getHead();
    tempMarker.pose.orientation = rpyToQuaternion(0, -90, 0);
    tempMarker.scale.x = 1;
    tempMarker.scale.y = 1;
    tempMarker.scale.z = 1;
    tempMarker.color.r = 0;
    tempMarker.color.g = 1;
    tempMarker.color.b = 0;
    tempMarker.color.a = 1;
    markerArrayToolpath.markers.push_back(tempMarker);

    // End marker.
    tempMarker.id = markerId++;
    tempMarker.pose.position = contour->getTail();
    tempMarker.scale.x = 1;
    tempMarker.scale.y = 1;
    tempMarker.scale.z = 1;
    tempMarker.color.r = 1;
    tempMarker.color.g = 0;
    tempMarker.color.b = 0;
    tempMarker.color.a = 1;
    markerArrayToolpath.markers.push_back(tempMarker);

    // Toolpath.
    tempMarker.id = markerId++;
    tempMarker.type = 4; // line strip.
    tempMarker.pose = geometry_msgs::msg::Pose();
    tempMarker.color.r = 0;
    tempMarker.color.g = 0;
    tempMarker.color.b = 1;
    tempMarker.color.a = 1;
    tempMarker.scale.x = 0.1;
    tempMarker.scale.y = 0.1;
    tempMarker.scale.z = 0.1;
    geometry_msgs::msg::PoseArray path = contour->getPath();

    for (geometry_msgs::msg::Pose &pose : path.poses) {
      tempMarker.points.push_back(pose.position);
    }
    
    markerArrayToolpath.markers.push_back(tempMarker);
  }

  // Publish visualization
  // TO DO

  // Perform TSP vs publish toolpaths
  // TO DO
}

geometry_msgs::msg::Quaternion PicassoEyes::rpyToQuaternion(const double roll, const double pitch, const double yaw) {
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  geometry_msgs::msg::Quaternion q_msg;
  tf2::convert(q_msg, q);
  return q_msg;
}