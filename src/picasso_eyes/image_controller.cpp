#include "image_controller.h"

imageController::imageController(const realsense2_camera_msgs::msg::RGBD::SharedPtr incomingMsg) : msgCameraimage_(*incomingMsg) {
  lsd_ = cv::createLineSegmentDetector();
}

void imageController::updateCameraImage(const realsense2_camera_msgs::msg::RGBD::SharedPtr incomingMsg) {
  std::unique_lock<std::mutex> lck(mutex_);
  msgCameraimage_ = *incomingMsg;
  cv::Mat originalImage = msg2Mat(msgCameraimage_.rgb);
  lck.unlock();

  if (originalImage.empty() || detectionRunning_) {
    return;
  }

  detectionRunning_ = true;
  imageProcessThread_ = std::thread(&imageController::generateArt, this, cv::Mat(originalImage));
  imageProcessThread_.detach();
}

void imageController::generateArt(cv::Mat image) {
  // image into R, G and B channels.
  cv::Mat channels[3];
  std::vector<std::vector<cv::Point>> contoursGroup[3];
  std::vector<cv::Vec4i> hierarchyGroup[3];
  cv::split(image, channels);
  cv::Mat contoursImage = cv::Mat(image.size(), false);
  cv::Mat testImage[3];

  for (int i = 0; i < 3; i++) {
    // Remove noise
    cv::GaussianBlur(channels[i], channels[i], cv::Size(3, 3), 0);

    // Detect contours
    detectContour(channels[i], contoursGroup[i], hierarchyGroup[i]);

    // Draw contours
    cv::Scalar colour = {0, 0, 0};
    colour[i] = 255;
    int thickness = 1;

    cv::drawContours(testImage[i], contoursGroup[i], -1, colour, thickness, 8, hierarchyGroup[i]);
  }

  displayImage(testImage[1]);
  displayImage(testImage[2]);
  displayImage(testImage[3]);

  //cv::destroyAllWindows();
  //displayImage(contoursImage);
  detectionRunning_ = false;
}

void imageController::displayImage(cv::Mat &image) {
  cv::imshow(testWin, image);
  cv::waitKey(0);
}

cv::Mat imageController::msg2Mat(sensor_msgs::msg::Image &imageMsg) {
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(imageMsg, imageMsg.encoding);
  cv::Mat image = cv_ptr->image;
  return image;
}

void imageController::edgeDetection(cv::Mat &image, float thresh) {
  // If image is multichannel, assuming has not been processed.
  if (image.channels() > 1) {
    cv::cvtColor(image, image, cv::COLOR_RGB2GRAY);
    cv::GaussianBlur(image, image, cv::Size(3, 3), 0);
  }

  int ave = findAverageIntensity(image);

  // Get canny values.
  int lower = round(std::max(double(0), (1.0 - thresh) * ave));
  int upper = round(std::min(double(255), (1.0 + thresh) * ave));

  // Find edges
  cv::Canny(image, image, lower, upper);
}

void imageController::detectContour(cv::Mat &image, std::vector<std::vector<cv::Point>> &contours, std::vector<cv::Vec4i> &hierarchy) {
  // If image is multichannel, assuming has not been processed.
  if (image.channels() > 1) {
    cv::cvtColor(image, image, cv::COLOR_RGB2GRAY);
    cv::GaussianBlur(image, image, cv::Size(3, 3), 0);
  }

  int ave = findAverageIntensity(image);
  threshold(image, image, ave, 255, cv::THRESH_BINARY);

  findContours(image, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
}

int imageController::findAverageIntensity(cv::Mat &image) {
  // If image is multichannel, assuming has not been processed.
  if (image.channels() > 1) {
    cv::cvtColor(image, image, cv::COLOR_RGB2GRAY);
    cv::GaussianBlur(image, image, cv::Size(3, 3), 0);
  }

  // Find average pixel value.
  double ave = 0;
  for (int y = 0; y < image.rows; y++) {
    for (int x = 0; x < image.cols; x++) {    
      ave += image.at<char>(x, y);
    }
  }

  ave /= image.total();

  return round(ave);
}
