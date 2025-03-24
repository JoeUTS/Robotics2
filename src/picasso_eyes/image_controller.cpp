#include "image_controller.h"

imageController::imageController(const std::shared_ptr<rclcpp::Node> owningNode, const sensor_msgs::msg::Image::SharedPtr incomingMsg) : msgCameraimage_(*incomingMsg), owningNode_(owningNode) {
  
}

void imageController::updateCameraImage(const sensor_msgs::msg::Image::SharedPtr incomingMsg) {
  std::unique_lock<std::mutex> lck(mutex_);
  msgCameraimage_ = *incomingMsg;
  lck.unlock();
  
  displayImage(msgCameraimage_);
}


void imageController::displayImage(sensor_msgs::msg::Image &imageMsg) {
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(imageMsg, "rgb8");
  //cv::patchNaNs(cv_ptr->image, 0.0);  // might only be needed for depth
  cv::Mat image = cv_ptr->image;

  if (image.empty()) {
    RCLCPP_WARN(owningNode_->get_logger(), "Cannot display empty image.");
    return;
  }

  cv::imshow("image", image);
}

/*
  // Load image
  cv::Mat imageOriginal = cv::imread(image_path);
  if (imageOriginal.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Could not load image from path");
    return;
  }

  // Convert to grayscale
  cv::Mat imageGray;
  cv::cvtColor(imageOriginal, imageGray, cv::COLOR_BGR2GRAY);

  // Detect lines
  std::vector<cv::Vec4i> lines;
  lsd_->detect(imageGray, lines);

  // Draw lines
  cv::Mat outputLines(imageOriginal.rows, imageOriginal.cols, CV_8UC3, cv::Scalar(0, 0, 0));
  lsd_->drawSegments(outputLines, lines);

  // Draw combined image
  cv::Mat outputCombined = imageOriginal.clone();
  lsd_->drawSegments(outputCombined, lines);

  cv::imshow("Loaded Image", imageOriginal);
  cv::imshow("Output Grey", outputLines);
  cv::imshow("Output Combined", outputCombined);

  cv::waitKey(0);
  cv::destroyAllWindows();
  */