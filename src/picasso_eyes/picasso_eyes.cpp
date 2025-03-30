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

  for (std::shared_ptr<Contour> contour : toolPaths) {
    visualization_msgs::msg::Marker markerStart, markerEnd;
    markerStart.type = visualization_msgs::msg::Marker::ARROW;
    // other stuff
    markerArrayToolpath.markers.push_back(markerStart);

    markerEnd.type = visualization_msgs::msg::Marker::ARROW;
    // other stuff
    markerArrayToolpath.markers.push_back(markerEnd);

    geometry_msgs::msg::PoseArray toolpath = contour->getPath();
    geometry_msgs::msg::Pose prevPose;
    for (geometry_msgs::msg::Pose pose : toolpath.poses) {
      // skip first iteration.
      if (!prevPose) {
        prevPose = pose;
        continue;
      }

      visualization_msgs::msg::Marker markerLine;
      markerLine.type = visualization_msgs::msg::Marker::LINE_STRIP;
      // other stuff
      markerArrayToolpath.markers.push_back(markerLine);
    }
  }
  
  // get start and end points from toolpaths

  visualization_msgs::msg::Marker markerToolpath;
  // get points from toolpaths

  // Publish visualization
  // TO DO

  // Perform TSP vs publish toolpaths
  // TO DO
}