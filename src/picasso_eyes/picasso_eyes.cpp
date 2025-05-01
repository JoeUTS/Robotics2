#include "picasso_eyes.h"

PicassoEyes::PicassoEyes(void) : Node("picaso_eyes") {
  // subscribers
  subCamera_ = this->create_subscription<realsense2_camera_msgs::msg::RGBD>(
    "/camera/camera/rgbd", 3, std::bind(&PicassoEyes::callbackCameraReceive, 
    this, std::placeholders::_1));

  // publishers
  pubVis_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/vis_toolpath", 3);
  pubCameraImage_ = this->create_publisher<sensor_msgs::msg::Image>("/processed_camera_image", 3);
  
  // timers
  timer_ = this->create_wall_timer(timer_duration_, std::bind(&PicassoEyes::tempFunction, this));

  // services
  servCameraToggleRepub_ = this->create_service<std_srvs::srv::Trigger>("/camera_feed_toggle", 
                                          std::bind(&PicassoEyes::serviceToggleCameraFeed, 
                                          this, std::placeholders::_1, std::placeholders::_2));

  servCaptureImage_ = this->create_service<std_srvs::srv::Trigger>("/capture_image", 
                                          std::bind(&PicassoEyes::serviceCaptureImage, 
                                          this, std::placeholders::_1, std::placeholders::_2)); 

  servPreviewSketch_ = this->create_service<std_srvs::srv::Trigger>("/preview_sketch", 
                                          std::bind(&PicassoEyes::servicePreviewSketch, 
                                          this, std::placeholders::_1, std::placeholders::_2)); 

  servDiscardImage_ = this->create_service<std_srvs::srv::Trigger>("/discard_image", 
                                          std::bind(&PicassoEyes::servicePreviewSketch, 
                                          this, std::placeholders::_1, std::placeholders::_2)); 

  servDrawSketch_ = this->create_service<std_srvs::srv::Trigger>("/draw_sketch", 
                                          std::bind(&PicassoEyes::serviceDrawSketch, 
                                          this, std::placeholders::_1, std::placeholders::_2));

  servNextContour_ = this->create_service<std_srvs::srv::Trigger>("/next_contour", 
                                          std::bind(&PicassoEyes::serviceNextContour, 
                                          this, std::placeholders::_1, std::placeholders::_2));

  serviceShutdown_ = this->create_service<std_srvs::srv::Trigger>("/picasso_eyes/shutdown_node", 
                                          std::bind(&PicassoEyes::serviceShutdown, 
                                          this, std::placeholders::_1, std::placeholders::_2));
}

void PicassoEyes::callbackCameraReceive(const realsense2_camera_msgs::msg::RGBD::SharedPtr incomingMsg) {
  if (cameraFeedEnabled_) { // Camera feed toggle.

    if (imageController_ == NULL) { // First image received.
      imageController_ = std::make_shared<imageController>(incomingMsg, this->shared_from_this());
      
    } else {
      sensor_msgs::msg::Image cameraImage = imageController_->updateCameraImage(incomingMsg);
      sensor_msgs::msg::Image publishedImage;

      if (imageCaptured_) { // Image captured.
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
  capturedImageMsg_ = imageController_->getStoredImage().rgb;
  capturedImage_ = imageController_->msg2Mat(capturedImageMsg_);

  if (capturedImage_.empty()) {
    imageCaptured_ = false;
    response->success = false;
    RCLCPP_INFO(this->get_logger(), "Captured Image");
    
  } else {
    imageCaptured_ = true;
    response->success = true;
    RCLCPP_INFO(this->get_logger(), "Failed to capture image.");
  }
}

void PicassoEyes::servicePreviewSketch(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                        std_srvs::srv::Trigger::Response::SharedPtr response) {
  cv::Mat localImage = capturedImage_;
  generateSketch(localImage, 1, 3, 3, true);
  toolPaths_ = generateToolpath(localImage, true);

  // Perform TSP
  //if (salesmanSolver_ == NULL) {
    //salesmanSolver_ = std::make_shared<SalesmanSolver>(this->shared_from_this());
  //}

  //salesmanSolver_->setContourList(toolPaths);
  //salesmanSolver_->solve();
  //contourOrder_ = salesmanSolver_->getTravelOrder();
  
  if (!contourOrder_.empty() && !toolPaths_.empty()) {
    response->success = true;
    RCLCPP_INFO(this->get_logger(), "Sketch Generated");
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

void PicassoEyes::serviceDrawSketch(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                    std_srvs::srv::Trigger::Response::SharedPtr response) {
  // TO DO: Make function for same
  response->success = true;
  std::string state = cameraFeedEnabled_ ? "ON" : "OFF";
  RCLCPP_INFO(this->get_logger(), "Camera feed turned %s", state.c_str());
}

void PicassoEyes::serviceNextContour(const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response) {
  // TO DO: Make function for same
  response->success = true;
  std::string state = cameraFeedEnabled_ ? "ON" : "OFF";
  RCLCPP_INFO(this->get_logger(), "Camera feed turned %s", state.c_str());
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

std::map<int, std::shared_ptr<Contour>> PicassoEyes::generateToolpath(cv::Mat image, const bool visualise) {
  std::map<int, std::shared_ptr<Contour>> toolPaths;

  if (image.empty()) {
    RCLCPP_WARN(this->get_logger(), "Failed to generate toolpath: empty image");
    return toolPaths;
  }

  if (image.dims > 1) { // Assume sketch not generated.
    image = generateSketch(image);
  }

  toolPaths = imageController_->getToolpaths(image);

  return toolPaths;
}

cv::Mat PicassoEyes::generateSketch(cv::Mat image, const int blurPasses, const int blurKernalSize, const int colourSteps, const bool showEdges) {
  std::string errorStart = "Failed to generate sketch: ";
  if (image.empty()) {
    RCLCPP_WARN(this->get_logger(), "%s empty image", errorStart.c_str());
    return cv::Mat();
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

  if (showEdges) {
    cv::imshow("Sketch Preview", edges);
    cv::waitKey(1);
  }

  std::map<int, std::shared_ptr<Contour>> toolPaths = generateToolpath(image, true, blurPasses, blurKernalSize, colourSteps, showEdges);

  if (toolPaths.empty()) {
    RCLCPP_WARN(this->get_logger(), "%s empty toolpaths", errorStart.c_str());
    return cv::Mat();
  }

  return edges;
}

void PicassoEyes::tempFunction(void) {
  cv::Mat imageRGB = imageController_->msg2Mat(imageController_->getStoredImage().rgb);

  //cv::Mat imageRGB = imread("local/resized_test_istockphoto.jpg", cv::IMREAD_UNCHANGED);

  if (imageRGB.empty()) {
    return;
  }

  // Function settings.
  // TO DO: Move elsewhere.
  std_msgs::msg::ColorRGBA colourHead;
  colourHead.r = 0.0;
  colourHead.g = 1.0;
  colourHead.b = 0.0;
  colourHead.a = 1.0;

  std_msgs::msg::ColorRGBA colourTail;
  colourTail.r = 1.0;
  colourTail.g = 0.0;
  colourTail.b = 0.0;
  colourTail.a = 1.0;

  std_msgs::msg::ColorRGBA colourPath;
  colourPath.r = 1.0;
  colourPath.g = 1.0;
  colourPath.b = 1.0;
  colourPath.a = 1.0;

  geometry_msgs::msg::Vector3 scaleHead;
  scaleHead.x = 0.001;
  scaleHead.y = 0.001;
  scaleHead.z = 0.02;

  geometry_msgs::msg::Vector3 scaleTail;
  scaleTail.x = 0.002;
  scaleTail.y = 0.002;
  scaleTail.z = 0.01;

  geometry_msgs::msg::Vector3 scaleLine;
  scaleLine.x = 0.001;

  // Localise people.
  // TO DO
  // Spin off new thread for this?
  //cv::Mat detectionImage = imageRGB.clone();
  //std::vector<DetectedObject> detectedObjects = imageController_->detect(detectionImage);
  /*
  std::vector<DetectedObject> detections = imageController_->detectSegment(detectionImage);
  for (const auto& obj : detections) {
    // You can now use obj.mask (a binary cv::Mat) for further processing
    // For example, to apply the mask to the original image region:
    cv::Mat object_region = detectionImage(obj.box);
    cv::Mat masked_object;
    object_region.copyTo(masked_object, obj.mask); // Apply the binary mask
    cv::imshow("Masked Object", masked_object);
  }
  */
  //std::vector<cv::Mat> detections = imageController_->detectPreProcess(detectionImage);
  //cv::Mat img = imageController_->detectPostProcess(detectionImage, detections);
  //cv::imshow("image", detectionImage);
  //cv::waitKey(1);

  // - Return an array of detected people with location, size, bounding box ect.
  std::list<geometry_msgs::msg::Vector3> personImageLocations;

  // Remove background
  // TO DO

  // Generate toolpaths
  const int blurPasses = 1;
  const int blurKernalSize = 3;
  const int colourSteps = 3;
  std::map<int, std::shared_ptr<Contour>> toolPaths = generateToolpath(imageRGB, true, blurPasses, blurKernalSize, colourSteps);

  if (toolPaths.empty()) {
    RCLCPP_WARN(this->get_logger(), "Empty toolpaths");
    return;
  }

  // Generate toolpaths visualization
  visualization_msgs::msg::MarkerArray markerArrayToolpath;
  unsigned int markerId = 0;

  for (auto &[key, contour] : toolPaths) {
    geometry_msgs::msg::Pose poseHead;
    poseHead.position.x = contour->getHead()->x;
    poseHead.position.y = contour->getHead()->y;
    addMarkerPoint(markerArrayToolpath, markerId++, poseHead, scaleHead, colourHead);

    geometry_msgs::msg::Pose poseTail;
    poseTail.position.x = contour->getTail()->x;
    poseTail.position.y = contour->getTail()->y;
    addMarkerPoint(markerArrayToolpath, markerId++, poseTail, scaleTail, colourTail);
    
    int sizeBefore = markerArrayToolpath.markers.size();
    std::vector<geometry_msgs::msg::Point> path = contour->getPoints();
    addMarkerPath(markerArrayToolpath, markerId++, path, scaleLine, colourPath);

    int sizeDelta = markerArrayToolpath.markers.size() - sizeBefore;

    if (sizeDelta > 1) {
      RCLCPP_WARN(this->get_logger(), "Path working");
    }
  }

  // Publish visualization
  if (markerArrayToolpath.markers.empty()) {
    RCLCPP_WARN(this->get_logger(), "Empty markers");
    return;
  }

  pubVis_->publish(markerArrayToolpath);
  
  // Perform TSP
  //if (salesmanSolver_ == NULL) {
    //salesmanSolver_ = std::make_shared<SalesmanSolver>(this->shared_from_this());
  //}

  //salesmanSolver_->setContourList(toolPaths);
  //salesmanSolver_->solve();
  //salesmanSolver_->getTravelOrder();
  
  // Publish pose array vs send via service
  
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