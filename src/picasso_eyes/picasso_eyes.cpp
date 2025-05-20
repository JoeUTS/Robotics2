#include "picasso_eyes.h"

PicassoEyes::PicassoEyes(void) : Node("picaso_eyes") {
  // subscribers
  subCamera_ = this->create_subscription<realsense2_camera_msgs::msg::RGBD>(
    "/camera/camera/rgbd", 3, std::bind(&PicassoEyes::callbackCameraReceive, 
    this, std::placeholders::_1));

  // publishers
  pubVis_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/vis_toolpath", 3);
  pubCameraImage_ = this->create_publisher<sensor_msgs::msg::Image>("/processed_camera_image", 3);

  // services
  servCameraToggleRepub_ = this->create_service<std_srvs::srv::Trigger>("/camera_feed_toggle", 
                                          std::bind(&PicassoEyes::serviceToggleCameraFeed, 
                                          this, std::placeholders::_1, std::placeholders::_2));

  servCaptureImage_ = this->create_service<std_srvs::srv::Trigger>("/capture_image", 
                                          std::bind(&PicassoEyes::serviceCaptureImage, 
                                          this, std::placeholders::_1, std::placeholders::_2)); 

  servPreviewSketch_ = this->create_service<picasso_bot::srv::GetImage>("/preview_sketch", 
                                          std::bind(&PicassoEyes::servicePreviewSketch, 
                                          this, std::placeholders::_1, std::placeholders::_2)); 

  servDiscardImage_ = this->create_service<std_srvs::srv::Trigger>("/discard_image", 
                                          std::bind(&PicassoEyes::serviceDiscardImage, 
                                          this, std::placeholders::_1, std::placeholders::_2)); 

  servGenerateToolpath_ = this->create_service<std_srvs::srv::Trigger>("/generate_toolpath", 
                                          std::bind(&PicassoEyes::serviceGenerateToolpath, 
                                          this, std::placeholders::_1, std::placeholders::_2));

  servNextContour_ = this->create_service<picasso_bot::srv::GetPoseArray>("/next_contour", 
                                          std::bind(&PicassoEyes::serviceNextContour, 
                                          this, std::placeholders::_1, std::placeholders::_2));

  servTotalLines_ = this->create_service<picasso_bot::srv::GetTotalLines>("/total_lines", 
                                          std::bind(&PicassoEyes::serviceGetTotalLines, 
                                          this, std::placeholders::_1, std::placeholders::_2));
                                          

  serviceShutdown_ = this->create_service<std_srvs::srv::Trigger>("/shutdown_node", 
                                          std::bind(&PicassoEyes::serviceShutdown, 
                                          this, std::placeholders::_1, std::placeholders::_2));
}

void PicassoEyes::callbackCameraReceive(const realsense2_camera_msgs::msg::RGBD::SharedPtr incomingMsg) {
  if (imageController_ == NULL) { // First image received.
    imageController_ = std::make_shared<imageController>(this->shared_from_this(), incomingMsg);
    
  } else {
    sensor_msgs::msg::Image cameraImage = imageController_->updateCameraImage(incomingMsg);

    if (cameraFeedEnabled_) { // Camera feed toggle.
      sensor_msgs::msg::Image publishedImage;

      if (imageCaptured_) {
        publishedImage = capturedImageMsg_;

      } else {
        publishedImage = cameraImage;
      }

      compressImage(publishedImage, pubCompressQuality_);
      pubCameraImage_->publish(publishedImage);
    }
  }
}

void PicassoEyes::serviceToggleCameraFeed(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                          std_srvs::srv::Trigger::Response::SharedPtr response) {
  cameraFeedEnabled_ = !cameraFeedEnabled_;
  response->success = true;
  std::string state = cameraFeedEnabled_ ? "ON" : "OFF";
  RCLCPP_INFO(this->get_logger(), "Camera feed turned %s", state.c_str());
}

void PicassoEyes::serviceCaptureImage(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                      std_srvs::srv::Trigger::Response::SharedPtr response) {
  // Image capture
  if (imageController_ == NULL) {
    RCLCPP_ERROR(this->get_logger(), "imageController is not initialized. Has a camera image been received yet?");
    response->success = false;
    return;
  }

  capturedImageMsg_ = imageController_->getStoredImage().rgb;
  capturedImage_ = imageController_->msg2Mat(capturedImageMsg_);
  
  // Mask generation
  /*
  if (!maskGenerationActive_) {
    maskGenerationActive_ = true;
    maskThread_ =  std::unique_ptr<std::thread>(new std::thread(&imageController::generateMask, imageController_, capturedImage_));

  } else {
    RCLCPP_WARN(this->get_logger(), "Cannot generate mask: Mask Generation already active.");
  }
  */

  if (capturedImage_.empty()) {
    imageCaptured_ = false;
    response->success = false;
    RCLCPP_INFO(this->get_logger(), "Failed to capture image.");
    
  } else {
    imageCaptured_ = true;
    response->success = true;
    RCLCPP_INFO(this->get_logger(), "Captured Image");
  }
}

void PicassoEyes::servicePreviewSketch(const picasso_bot::srv::GetImage::Request::SharedPtr request, 
                                      picasso_bot::srv::GetImage::Response::SharedPtr response) {
  cv::Mat localImage = capturedImage_.clone();
  localImage = generateSketch(localImage, 1, 3, 3);
  
  if (localImage.empty()) {
    response->success = false;
    RCLCPP_INFO(this->get_logger(), "Failed to generate sketch.");
    
  } else {
    // convert to image msg
    cv_bridge::CvImage imgBridge = cv_bridge::CvImage(std_msgs::msg::Header(), 
                                                      sensor_msgs::image_encodings::MONO8, 
                                                      localImage);
    
    // Send responce
    imgBridge.toImageMsg(response->image);
    response->success = true;
    RCLCPP_INFO(this->get_logger(), "Sketch generated successfully.");
  }
}

void PicassoEyes::serviceDiscardImage(const std_srvs::srv::Trigger::Request::SharedPtr request, 
                              std_srvs::srv::Trigger::Response::SharedPtr response) {
  capturedImage_ = cv::Mat();
  capturedImageMsg_ = sensor_msgs::msg::Image();
  imageCaptured_ = false;
  toolPaths_.clear();
  contourOrder_.clear();

  response->success = true;
  RCLCPP_INFO(this->get_logger(), "Discarded captured image.");
}

void PicassoEyes::serviceGenerateToolpath(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                    std_srvs::srv::Trigger::Response::SharedPtr response) {
  contourOrder_.clear();
  contourOrderIndex_ = 0;
  std::string errorStart = "Cannot generate toolpath:";
  cv::Mat localImage = capturedImage_.clone();

  if (localImage.empty()) {
    RCLCPP_WARN(this->get_logger(), "%s empty image", errorStart.c_str());
    response->success = false;
    return;
  }

  localImage = generateSketch(localImage, 1, 3, 3);
  toolPaths_ = generateToolpath(localImage, true);

  if (toolPaths_.empty()) {
    RCLCPP_WARN(this->get_logger(), "%s empty toolpaths", errorStart.c_str());
    response->success = false;
    return;
  }

  // Perform TSP
  RCLCPP_INFO(this->get_logger(), "Solving TSP...");

  if (salesmanSolver_ == NULL) {
    salesmanSolver_ = std::make_shared<SalesmanSolver>(this->shared_from_this());
  }

  salesmanSolver_->setContourList(toolPaths_);
  std::chrono::time_point<std::chrono::system_clock> startTime = std::chrono::system_clock::now();
  salesmanSolver_->solve();
  std::chrono::duration<double> duration = std::chrono::system_clock::now() - startTime;
  double solveTime = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();

  contourOrder_.reserve(toolPaths_.size());
  contourOrder_ = salesmanSolver_->getTravelOrder();
  
  if (contourOrder_.empty()) {
    RCLCPP_WARN(this->get_logger(), "%s empty draw order", errorStart.c_str());
    response->success = false;
    
  } else {
    response->success = true;
    RCLCPP_INFO(this->get_logger(), "Toolpath generated successfully, solve time: %.2f ms", solveTime / 1000);
  }

  return;
}

void PicassoEyes::serviceGetTotalLines(const picasso_bot::srv::GetTotalLines::Request::SharedPtr request, picasso_bot::srv::GetTotalLines::Response::SharedPtr response) {
  std::string errorStart = "Cannot get total lines:";
  if (contourOrder_.empty()) {
    response->success = false;
    response->amount = 0;
    RCLCPP_WARN(this->get_logger(), "%s no toolpath generated!", errorStart.c_str());

    return;

  } else {
    response->success = true;
    response->amount = contourOrder_.size();
  }
}

void PicassoEyes::serviceNextContour(const picasso_bot::srv::GetPoseArray::Request::SharedPtr request, 
                                      picasso_bot::srv::GetPoseArray::Response::SharedPtr response) {
  std::string errorStart = "Cannot get next contour:";

  if (contourOrder_.empty() || contourOrderIndex_ == contourOrder_.size() - 1) {
    response->success = false;
    response->poses = geometry_msgs::msg::PoseArray();

    if (contourOrder_.empty()) {
      RCLCPP_WARN(this->get_logger(), "%s empty draw order!", errorStart.c_str());

    } else {
      RCLCPP_INFO(this->get_logger(), "%s contour complete!", errorStart.c_str());
    }
    
  } else {
    response->success = true;
    std::shared_ptr<Contour> contour = toolPaths_.at(contourOrder_.at(contourOrderIndex_).second);
    response->poses = contour.get()->getPath(); // NEED TO ALLOW FOR BACKWARDS PATHS!!
    contourOrderIndex_++;
    RCLCPP_INFO(this->get_logger(), "Next contour retrieved.");
  }
}

void PicassoEyes::serviceShutdown(const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response) {
  response->success = true;
  RCLCPP_INFO(this->get_logger(), "%s shutting down.", this->get_name());

  rclcpp::shutdown(); 
}

sensor_msgs::msg::Image PicassoEyes::compressImage(sensor_msgs::msg::Image &imageMsg, const int quality) {
  std::vector<uchar> buffer;
  std::vector<int> compression_params;
  compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
  compression_params.push_back(quality);
  cv::imencode(".jpg", imageController_->msg2Mat(imageMsg), buffer, compression_params);

  sensor_msgs::msg::Image compressedMsg = sensor_msgs::msg::Image();
  compressedMsg.header = imageMsg.header;
  compressedMsg.height = imageMsg.height;
  compressedMsg.width = imageMsg.width;
  compressedMsg.encoding = "jpeg";
  compressedMsg.is_bigendian = 0;
  compressedMsg.step = 0;
  compressedMsg.data.assign(buffer.begin(), buffer.end());
  
  return compressedMsg;
}

std::map<int, std::shared_ptr<Contour>> PicassoEyes::generateToolpath(cv::Mat &image, const bool visualise) {
  std::map<int, std::shared_ptr<Contour>> toolPaths;

  if (image.empty()) {
    RCLCPP_WARN(this->get_logger(), "Failed to generate toolpath: empty image");
    return toolPaths;
  }

  if (image.dims > 1) { // Assume sketch not generated.
    image = generateSketch(image);
  }

  toolPaths = imageController_->getToolpaths(image);

  if (visualise) {
    visualization_msgs::msg::MarkerArray markerArrayToolpath;
    unsigned int markerId = 0;

    // Generate toolpaths visualization
    for (auto &[key, contour] : toolPaths) {
      geometry_msgs::msg::Pose poseHead;
      poseHead.position.x = contour->getHead()->x;
      poseHead.position.y = contour->getHead()->y;
      addMarkerPoint(markerArrayToolpath, markerId++, poseHead, Scales::scaleHead, Colours::green);

      geometry_msgs::msg::Pose poseTail;
      poseTail.position.x = contour->getTail()->x;
      poseTail.position.y = contour->getTail()->y;
      addMarkerPoint(markerArrayToolpath, markerId++, poseTail, Scales::scaleTail, Colours::red);

      std::vector<geometry_msgs::msg::Point> path = contour->getPoints();
      addMarkerPath(markerArrayToolpath, markerId++, path, Scales::scaleLine, Colours::white);
    }

    // Publish visualization
    if (markerArrayToolpath.markers.empty()) {
      RCLCPP_WARN(this->get_logger(), "Empty markers");

    } else {
      pubVis_->publish(markerArrayToolpath);
    }
  }

  return toolPaths;
}

cv::Mat PicassoEyes::generateSketch(cv::Mat &image, const int blurPasses, const int blurKernalSize, const int colourSteps, const bool showEdges) {
  std::string errorStart = "Failed to generate sketch: ";
  if (image.empty()) {
    RCLCPP_WARN(this->get_logger(), "%s empty image", errorStart.c_str());
    return cv::Mat();
  }

  // Remove background
  // TO DO

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

  if (showEdges) {
    cv::imshow("Sketch Preview", edges);
    cv::waitKey(1);
  }

  return edges;
}

geometry_msgs::msg::Quaternion PicassoEyes::rpyToQuaternion(const double roll, 
                                                            const double pitch, 
                                                            const double yaw) {
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  geometry_msgs::msg::Quaternion q_msg;
  tf2::convert(q, q_msg);
  return q_msg;
}

void PicassoEyes::addMarkerPoint(visualization_msgs::msg::MarkerArray &markerArray, 
                                const unsigned int id, const geometry_msgs::msg::Pose &pose, 
                                const geometry_msgs::msg::Vector3 &scale, 
                                const std_msgs::msg::ColorRGBA &colour) {
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

void PicassoEyes::addMarkerPath(visualization_msgs::msg::MarkerArray &markerArray, 
                                const unsigned int id, 
                                const std::vector<geometry_msgs::msg::Point> &points, 
                                const geometry_msgs::msg::Vector3 &scale, 
                                const std_msgs::msg::ColorRGBA &colour) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "camera_link";
  marker.header.stamp = this->get_clock()->now();
  marker.ns = "toolpath";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.points = points;
  
  marker.scale = scale;
  marker.color = colour;

  markerArray.markers.push_back(marker);
}


std::vector<cv::Point> PicassoEyes::convertToCvPoints(const std::vector<geometry_msgs::msg::Point> &rosPoints) {
  std::vector<cv::Point> cvPoints;
  for (const auto &rosPoint : rosPoints) {
      cvPoints.emplace_back(cv::Point(static_cast<int>(rosPoint.x), static_cast<int>(rosPoint.y)));
  }
  return cvPoints;
}

void PicassoEyes::generateMask(cv::Mat &image) {
  // TO DO: Finish
  if (maskGenerationActive_) {
    RCLCPP_WARN(this->get_logger(), "Cannot generate mask: Mask Generation already active.");
    return;
  } 

  maskGenerationActive_ = true;
  RCLCPP_INFO(this->get_logger(), "Mask generation started.");
  // TO DO: Add calculate processing time.
  cv::Mat detectionImage = image.clone();
  maskThread_ =  std::unique_ptr<std::thread>(new std::thread(&imageController::generateMask, imageController_, detectionImage));

  maskReady_ = false;
  maskGenerationActive_ = false;
}