#ifndef CONTOUR_H
#define CONTOUR_H

#include <vector>
#include <list>
#include <memory>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <opencv2/opencv.hpp>

class Contour : public std::enable_shared_from_this<Contour> {
public:
  static std::shared_ptr<Contour> create(const int contourID, std::vector<cv::Point> &contours) {
    std::shared_ptr<Contour> instance = std::make_shared<Contour>(contourID);
    instance->initialize(contours);
    return instance;
  }

  // Constructs contour object from a vector of cv::Points
  Contour(const int contourID) : contourID_(contourID) {};

  // Gets contour's ID
  int getID(void) {return contourID_;};

  // Gets drawn status.
  bool getDrawn(void) {return isDrawn_;};

  // Sets drawn status.
  void setDrawn(void) {isDrawn_ = true;};

  // Gets contour head point.
  geometry_msgs::msg::Point getHead(void) {return *head_;};

  // Gets contour tail point.
  geometry_msgs::msg::Point getTail(void) {return *tail_;};

  // Gets forward contour path.
  geometry_msgs::msg::PoseArray getPath(void) {
    geometry_msgs::msg::PoseArray poseArray;
    poseArray.poses.reserve(points_.size());
    std::for_each(points_.begin(), points_.end(), [this, &poseArray](geometry_msgs::msg::Point &point) {
      geometry_msgs::msg::Pose pose;
      pose.position = point;
      poseArray.poses.push_back(pose);
    });

    return poseArray;
  };

  // Gets reverse contour path.
  geometry_msgs::msg::PoseArray getPathBackwards(void) {
    geometry_msgs::msg::PoseArray poseArray;
    poseArray.poses.reserve(points_.size());
    std::for_each(points_.end(), points_.begin(), [this, &poseArray](geometry_msgs::msg::Point &point) {
      geometry_msgs::msg::Pose pose;
      pose.position = point;
      poseArray.poses.push_back(pose);
    });

    return poseArray;
  };

private:
  const int contourID_;                             // Contour ID.
  bool isDrawn_ = false;                            // True if contour is complete.
  std::list<geometry_msgs::msg::Point> points_;     // Contour points.
  
  std::shared_ptr<Contour> selfPtr_;                // Pointer to self.
  std::unique_ptr<geometry_msgs::msg::Point> head_; // Pointer to start point.
  std::unique_ptr<geometry_msgs::msg::Point> tail_; // Pointer to end point.

  // Initializes contour object via converting path.
  void initialize(std::vector<cv::Point> &contours) {
    // Convert points.
    std::for_each(contours.begin(), contours.end(), [this](cv::Point &point) {
      points_.emplace_back(cv2rosPoint(point));
    });

    // set pointers.
    head_ = std::make_unique<geometry_msgs::msg::Point>(points_.front());
    tail_ = std::make_unique<geometry_msgs::msg::Point>(points_.back());
    selfPtr_ = shared_from_this();
  }
  
  // Converts an openCV point to a ros one.
  geometry_msgs::msg::Point cv2rosPoint(cv::Point &cvPoint) {
    geometry_msgs::msg::Point point;
    point.x = static_cast<double>(cvPoint.x);
    point.y = static_cast<double>(cvPoint.y);
    point.z = 0.0;

    return point;
  }

};

#endif // CONTOUR_H