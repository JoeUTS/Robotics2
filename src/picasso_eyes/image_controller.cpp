#include "image_controller.h"

imageController::imageController(const realsense2_camera_msgs::msg::RGBD::SharedPtr incomingMsg) : msgCameraimage_(*incomingMsg) {
  lsd_ = cv::createLineSegmentDetector();
}

void imageController::updateCameraImage(const realsense2_camera_msgs::msg::RGBD::SharedPtr incomingMsg) {
  std::unique_lock<std::mutex> lck(mutex_);
  msgCameraimage_ = *incomingMsg;
  lck.unlock();

  if (!detectionRunning_) {
    detectionRunning_ = true;
    imageProcessThread_ = std::thread(&imageController::generateArt, this);
    imageProcessThread_.detach();
  }
}

void imageController::generateArt(void) {
  const int kernalSize = 3;

  std::unique_lock<std::mutex> lck(mutex_);
  cv::Mat imageRGB = msg2Mat(msgCameraimage_.rgb);
  cv::Mat imageDepth = msg2Mat(msgCameraimage_.depth);
  lck.unlock();

  if (imageRGB.empty() || imageDepth.empty()) {
    detectionRunning_ = false;
    return;
  }

  // Process images.
  // Filter depth.
  cv::Mat imageDepthBlur;
  cv::GaussianBlur(imageDepth, imageDepthBlur, cv::Size(kernalSize, kernalSize), 0);

  // Make greyscale of RGB.
  cv::Mat imageGrey;
  cv::cvtColor(imageRGB, imageGrey, cv::COLOR_BGR2GRAY);
  cv::GaussianBlur(imageGrey, imageGrey, cv::Size(kernalSize, kernalSize), 0);

  // Extract color channels.
  cv::Mat imageChannels[3];
  cv::split(imageRGB, imageChannels);

  for (int i = 0; i < 3; i++) {
    cv::GaussianBlur(imageChannels[i], imageChannels[i], cv::Size(kernalSize, kernalSize), 0);
  }

  // Find face in image and get location.
  // TO DO

  // Get depth at that location.
  // TO DO

  // Make depth masks.
  // TO DO

  // Remove background.
  // TO DO

  // Draw bounding box around face
  // TO DO

  // Detect contours for each channel + grey
  std::vector<std::vector<cv::Point>> contoursGroup[4];
  std::vector<cv::Vec4i> hierarchyGroup[4];
  cv::Mat imageChannelEdges[4];

  float thresh = 0.5;
  for (int i = 0; i < 3; i++) {
    imageChannelEdges[i] = imageChannels[i].clone();
    edgeDetection(imageChannels[i], thresh);
    detectContour(imageChannels[i], contoursGroup[i], hierarchyGroup[i]);
  }

  imageChannelEdges[3] = imageGrey.clone();
  edgeDetection(imageChannels[3], thresh);
  detectContour(imageChannelEdges[3], contoursGroup[3], hierarchyGroup[3]);

  // Draw contours.
  //cv::Mat imageContours = cv::Mat(imageGrey.size(), CV_8UC3, cv::Scalar(0));
  cv::Mat imageContours = imageRGB.clone();

  for (int i = 0; i < 3; i++) {
    cv::Scalar colour = {0, 0, 0};
    colour[i] = 255;
    cv::drawContours(imageContours, contoursGroup[i], -1, colour, 1, 8, hierarchyGroup[i]);
  }

  cv::drawContours(imageContours, contoursGroup[3], -1, cv::Scalar(255), 1, 8, hierarchyGroup[3]);

  cv::imshow("contours", imageContours);
  cv::waitKey(1);

  detectionRunning_ = false;
}

void imageController::displayImage(cv::Mat &image) {
  cv::imshow(testWin, image);
  cv::waitKey(0);
}

cv::Mat imageController::msg2Mat(sensor_msgs::msg::Image &imageMsg) {
  cv_bridge::CvImagePtr cvPtr;
  cv::Mat image;

  if (imageMsg.encoding == "16UC1" || imageMsg.encoding == "32FC1") {
    // Depth image.
    cvPtr = cv_bridge::toCvCopy(imageMsg);
    image = cvPtr->image;
    double min, max;
    cv::minMaxLoc(image, &min, &max);
    image.convertTo(image, CV_8UC1, 255.0 / max);

  } else if (imageMsg.encoding == "mono8") {
    // Grayscale image
    cvPtr = cv_bridge::toCvCopy(imageMsg, "mono8");
    image = cvPtr->image;

  } else {
    // Colour image.
    // NOTE: Camera encoding says RGB but is actually BGR.
    cvPtr = cv_bridge::toCvCopy(imageMsg, "bgr8");
    image = cvPtr->image;
  }

  return image;
}

void imageController::edgeDetection(cv::Mat &image, float thresh) {
  // If image is multichannel, assuming has not been processed.
  if (image.channels() > 1) {
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(image, image, cv::Size(3, 3), 0);
  }

  int ave = findAverageIntensity(image);

  // Get canny values.
  int lower = round(std::max(double(0), (1.0 - thresh) * ave));
  int upper = round(std::min(double(255), (1.0 + thresh) * ave));

  // Find edges
  cv::Canny(image, image, lower, upper);
}

void imageController::detectContour(cv::Mat image, std::vector<std::vector<cv::Point>> &contours, std::vector<cv::Vec4i> &hierarchy) {
  // If image is multichannel, assuming has not been processed.
  if (image.channels() > 1) {
    cv::cvtColor(image, image, cv::COLOR_RGB2GRAY);
    cv::GaussianBlur(image, image, cv::Size(3, 3), 0);
  }

  int ave = findAverageIntensity(image);
  cv::threshold(image, image, ave, 255, cv::THRESH_BINARY);

  findContours(image, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
}

int imageController::findAverageIntensity(cv::Mat &image) {
  // If image is multichannel, assuming has not been processed.
  if (image.channels() > 1) {
    cv::cvtColor(image, image, cv::COLOR_RGB2GRAY);
    cv::GaussianBlur(image, image, cv::Size(3, 3), 0);
  }

  cv::Scalar mean = cv::mean(image);
  return round(mean[0]);
}
