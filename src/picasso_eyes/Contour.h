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
  static std::shared_ptr<Contour> create(const int contourID, const cv::Mat &image, const float scale, std::vector<cv::Point> &contours) {
    std::shared_ptr<Contour> instance = std::make_shared<Contour>(contourID);
    instance->initialize(image, scale, contours);
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
  std::shared_ptr<geometry_msgs::msg::Point> getHead(void) {return points_.front();};

  // Gets contour tail point.
  std::shared_ptr<geometry_msgs::msg::Point> getTail(void) {return points_.back();};

  // Gets forward contour path.
  geometry_msgs::msg::PoseArray getPath(void) {
    geometry_msgs::msg::PoseArray poseArray;
    poseArray.poses.reserve(points_.size());

    for (auto it = points_.begin(); it != points_.end(); ++it) {
      geometry_msgs::msg::Pose pose;
      pose.position = **it;
      poseArray.poses.push_back(pose);
    }

    return poseArray;
  };

  // Gets reverse contour path.
  geometry_msgs::msg::PoseArray getPathBackwards(void) {
    geometry_msgs::msg::PoseArray poseArray;
    poseArray.poses.reserve(points_.size());

    for (auto it = points_.rbegin(); it != points_.rend(); ++it) {
      geometry_msgs::msg::Pose pose;
      pose.position = **it;
      poseArray.poses.push_back(pose);
    }

    return poseArray;
  };

  // Gets list of points.
  std::vector<geometry_msgs::msg::Point> getPoints(void) {
    std::vector<geometry_msgs::msg::Point> points;

    for (auto it = points_.begin(); it != points_.end(); ++it) {
      points.push_back(**it);
    }

    return points;
  };

private:
  const int contourID_;                             // Contour ID.
  bool isDrawn_ = false;                            // True if contour is complete.
  std::shared_ptr<Contour> selfPtr_;                // Pointer to self.
  std::list<std::shared_ptr<geometry_msgs::msg::Point>> points_;     // Contour points.

  // Initializes contour object via converting path.
  void initialize(const cv::Mat &image, const float scale, std::vector<cv::Point> &contours) {
    // Convert points.
    for (unsigned int i = 0; i < contours.size(); i++) {
      std::shared_ptr<geometry_msgs::msg::Point> pointMsg = std::make_shared<geometry_msgs::msg::Point>();
      
      // Center image.
      pointMsg->x -= image.cols / 2;
      pointMsg->y -= image.rows / 2;

      // Scale image.
      pointMsg->x *= scale;
      pointMsg->y *= scale;
      
      points_.push_back(pointMsg);
    }

    selfPtr_ = shared_from_this();
  }
};

#endif // CONTOUR_H