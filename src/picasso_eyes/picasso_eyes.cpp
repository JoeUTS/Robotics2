#include "picasso_eyes.h"

PicassoEyes::PicassoEyes(void) : Node("picaso_eyes") {
  // subscribers
  subCameraImage_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera/color/image_raw", 1000, std::bind(&PicassoEyes::cameraImageCallback,this,std::placeholders::_1));

}

void PicassoEyes::cameraImageCallback(const sensor_msgs::msg::Image::SharedPtr incomingMsg) {
  if (imageController_ == NULL) {
    RCLCPP_WARN(this->get_logger(), "First image!");
    imageController_ = std::make_shared<imageController>(std::shared_ptr<rclcpp::Node>(this), incomingMsg);
  } else {
    RCLCPP_WARN(this->get_logger(), "Subsiquent image!");
    imageController_->updateCameraImage(incomingMsg);
  }
}