#include "imageController.h"

imageController::imageController(const sensor_msgs::msg::Image::SharedPtr incomingMsg) :  msgCameraimage_(*incomingMsg) {
  
}

void imageController::updateCameraImage(const sensor_msgs::msg::Image::SharedPtr incomingMsg) {
  std::unique_lock<std::mutex> lck(mutex_);
  msgCameraimage_ = *incomingMsg;
  lck.unlock();
}