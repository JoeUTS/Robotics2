#include "image_controller.h"

imageController::imageController(const realsense2_camera_msgs::msg::RGBD::SharedPtr incomingMsg, std::shared_ptr<rclcpp::Node> parentNode) 
  : lastCameraMsg_(*incomingMsg), parentNode_(parentNode) {
  // Get paths to config files.
  std::string packageShareDir = ament_index_cpp::get_package_share_directory("picasso_bot");
  //std::string pathImageClasses = packageShareDir + "/config/coco-classes.txt";
  //std::string pathYOLO = packageShareDir + "/config/yolov8m-seg.onnx";

}

sensor_msgs::msg::Image imageController::updateCameraImage(const realsense2_camera_msgs::msg::RGBD::SharedPtr incomingMsg) {
  std::unique_lock<std::mutex> lck(mutex_);
  lastCameraMsg_ = *incomingMsg;
  sensor_msgs::msg::Image cameraImage = lastCameraMsg_.rgb;
  lck.unlock();

  return cameraImage;
}

realsense2_camera_msgs::msg::RGBD imageController::getStoredImage(void) {
  std::unique_lock<std::mutex> lck(mutex_);
  realsense2_camera_msgs::msg::RGBD msg = lastCameraMsg_;
  lck.unlock();

  return msg;
}

void imageController::editImageQuantize(cv::Mat &image, const int colourSteps) {
  // Prepare image.
  cv::Mat data;
  image.convertTo(data, CV_32F);
  data = data.reshape(1, data.total());  // Place each pixel in its own row.

  // Perform K-means clustering
  cv::Mat labels, centers;
  kmeans(data, colourSteps, labels, cv::TermCriteria(CV_TERMCRIT_ITER, 10, 1.0), 3, 
    cv::KMEANS_PP_CENTERS, centers);

  // Reshape each to row of RGB pixels
  centers = centers.reshape(3, centers.rows);
  data = data.reshape(3, data.rows);

  // Replace each pixel with cluster value.
  cv::Vec3f *pixels = data.ptr<cv::Vec3f>();
  for (size_t i = 0; i < data.rows; i++) {
    int centerId = labels.at<int>(i);
    pixels[i] = centers.at<cv::Vec3f>(centerId);
  }

  // Allocate back to input image.
  int imageType = image.type();
  image = data.reshape(image.channels(), image.rows);
  image.convertTo(image, imageType);
}

cv::Scalar imageController::getNextColour(void) {
  if (colourIndex_ >= COLOURS_LIST.size() - 1) {
    colourIndex_ = 0;
  }

  cv::Scalar colour = COLOURS_LIST.at(colourIndex_);
  colourIndex_++;

  return colour;
}

cv::Mat imageController::msg2Mat(const sensor_msgs::msg::Image &imageMsg) {
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

void imageController::detectEdges(cv::Mat &image, float thresh) {
  int ave = calculateAverageIntensity(image);

  // Get canny values.
  int lower = round(std::max(double(0), (1.0 - thresh) * ave));
  int upper = round(std::min(double(255), (1.0 + thresh) * ave));

  // Find edges
  cv::Canny(image, image, lower, upper);
}

std::map<int, std::shared_ptr<Contour>> imageController::getToolpaths(const cv::Mat &image, const bool normalise) {
  std::map<int, std::shared_ptr<Contour>> contoursList;

  // Find contours.
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(image, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
  int indexMod = 0;

  for (int i = 0; i < contours.size() - 1; i++) {
    if (contours.at(i).size() < 2) {
      indexMod++;
      continue;
    }

    std::shared_ptr<Contour> contourPtr = Contour::create(i - indexMod, contours.at(i), image, parentNode_);
    contoursList.insert(std::pair<int, std::shared_ptr<Contour>>(i - indexMod,  contourPtr));
  }

  return contoursList;
}

int imageController::calculateAverageIntensity(cv::Mat &image) {
  cv::Scalar mean = cv::mean(image);
  return round(mean[0]);
}

void imageController::generateArt(void) {
  const int kernalSize = 3;
  const int blurPasses = 1;
  const int colourSteps = 3;

  std::unique_lock<std::mutex> lck(mutex_);
  cv::Mat imageRGB = msg2Mat(lastCameraMsg_.rgb);
  lck.unlock();

  // Return on empty msg.
  if (imageRGB.empty()) {
    detectionRunning_ = false;
    return;
  }
  
  for (int i = 0; i < blurPasses; i++) {
    editImageBlurMedian(imageRGB, kernalSize);
    editImageBlurGaussian(imageRGB, kernalSize);
  }
  
  editImageQuantize(imageRGB, colourSteps);

  cv::Mat edges = imageRGB.clone();
  editImageGreyscale(edges);
  detectEdges(edges, 0.1);

  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(edges, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
  cv::drawContours(imageRGB, contours, -1, cv::Scalar(0, 0, 255));

  // Generate toolpaths.
  std::vector<std::shared_ptr<Contour>> toolPaths;
  toolPaths.reserve(contours.size());

  for (std::vector<cv::Point> &contour : contours) {
    //std::shared_ptr<Contour> contourPtr = Contour::create(toolPaths.size(), imageRGB, 1.0, contour);
    //toolPaths.push_back(contourPtr);
  }

  cv::imshow("output", edges);
  cv::waitKey(1);

  
  detectionRunning_ = false;
}