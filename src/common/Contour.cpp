# include "Contour.h"

Contour::Contour(const int contourID, std::vector<std::shared_ptr<geometry_msgs::msg::Point>> contourPoints) 
  : contourID_(contourID), contourPoints_(contourPoints) {};

int Contour::getID(void) {
  return contourID_;
};

std::shared_ptr<Contour> Contour::getContourPtr(void) {
  return shared_from_this();
};

bool Contour::getDrawn(void) {
  return isDrawn_;
};

void Contour::setDrawn(bool state) {
  isDrawn_ = state;
};

std::shared_ptr<geometry_msgs::msg::Point> Contour::getHead(void) {
  return contourPoints_.front();
};

std::shared_ptr<geometry_msgs::msg::Point> Contour::getTail(void) {
  return contourPoints_.back();
};

geometry_msgs::msg::PoseArray Contour::getPath(void) {
  geometry_msgs::msg::PoseArray poseArray;
  poseArray.poses.reserve(contourPoints_.size());

  for (auto it = contourPoints_.begin(); it != contourPoints_.end(); ++it) {
    geometry_msgs::msg::Pose pose;
    std::shared_ptr<geometry_msgs::msg::Point> pointPtr = *it;
    pose.position.x = pointPtr->x;
    pose.position.y = pointPtr->y;
    poseArray.poses.push_back(pose);
  }

  return poseArray;
};

geometry_msgs::msg::PoseArray Contour::getPathBackwards(void) {
  geometry_msgs::msg::PoseArray poseArray;
  poseArray.poses.reserve(contourPoints_.size());

  for (auto it = contourPoints_.rbegin(); it != contourPoints_.rend(); ++it) {
    geometry_msgs::msg::Pose pose;
    std::shared_ptr<geometry_msgs::msg::Point> pointPtr = *it;
    pose.position.x = pointPtr->x;
    pose.position.y = pointPtr->y;
    poseArray.poses.push_back(pose);
  }

  return poseArray;
};

std::vector<geometry_msgs::msg::Point> Contour::getPoints(void) {
  std::vector<geometry_msgs::msg::Point> points;
  points.reserve(contourPoints_.size());

  for (auto it = contourPoints_.begin(); it != contourPoints_.end(); ++it) {
    geometry_msgs::msg::Point point = **it;
    points.push_back(point);
  }

  return points;
};