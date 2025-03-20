#include "picasso_eyes.h"

PicassoEyes::PicassoEyes(void) : Node("picaso_eyes") {
  // subscribers
  subCameraImage_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera/camera/color/image_raw", 1000, std::bind(&PicassoEyes::cameraImageCallback,this,std::placeholders::_1));

}

PicassoEyes::~PicassoEyes(void) {

}

void PicassoEyes::cameraImageCallback(const sensor_msgs::msg::Image::SharedPtr incomingMsg) {
  if (imageController_ == NULL) {
    RCLCPP_INFO(this->get_logger(), "First image!");
    imageController_ = std::make_shared<imageController>(incomingMsg);
  } else {
    RCLCPP_INFO(this->get_logger(), "Subsiquent image!");
    imageController_->updateCameraImage(incomingMsg);
  }
}