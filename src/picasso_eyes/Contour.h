#ifndef CONTOUR_H
#define CONTOUR_H

#include <vector>
#include <list>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <opencv2/opencv.hpp>

class Contour : public std::enable_shared_from_this<Contour> {
public:
  static std::shared_ptr<Contour> create(const int contourID, std::vector<cv::Point> &contours, cv::Mat &image, std::shared_ptr<rclcpp::Node> parentNode = NULL) {
    std::shared_ptr<Contour> instance = std::make_shared<Contour>(contourID, parentNode);
    instance->initialize(image, contours);
    return instance;
  }

  /// @brief Constructor, sets ID. Not to be called directly, use Contour::create().
  /// @param contourID ID of contour.
  Contour(const int contourID, std::shared_ptr<rclcpp::Node> parentNode) : contourID_(contourID), parentNode_(parentNode) {};

  /// @brief ID getter.
  /// @return int. contourID_.
  int getID(void) {return contourID_;};

  /// @brief self pointer getter.
  /// @return std::shared_ptr<Contour>. selfPtr_.
  std::shared_ptr<Contour> getContourPtr(void) {return selfPtr_;};

  /// @brief is drawn getter.
  /// @return bool. isDrawn_.
  bool getDrawn(void) {return isDrawn_;};

  /// @brief Drawn status setter true.
  void setDrawn(void) {isDrawn_ = true;};

  /// @brief first point getter.
  /// @return std::shared_ptr<geometry_msgs::msg::Point>. points_.front().
  std::shared_ptr<geometry_msgs::msg::Point> getHead(void) {return points_.front();};

  /// @brief last point getter.
  /// @return std::shared_ptr<geometry_msgs::msg::Point>. points_.back().
  std::shared_ptr<geometry_msgs::msg::Point> getTail(void) {return points_.back();};

  /// @brief Gets points list from start to end as a PoseArray.
  /// @return geometry_msgs::msg::PoseArray. points_.
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

  /// @brief Gets points list from end to start as a PoseArray.
  /// @return geometry_msgs::msg::PoseArray. points_.
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

  /// @brief Get list of points as a vector of points.
  /// @return std::vector<geometry_msgs::msg::Point>. points_.
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
  std::shared_ptr<rclcpp::Node> parentNode_;
  std::list<std::shared_ptr<geometry_msgs::msg::Point>> points_;     // Contour points.

  // Initializes contour object via converting path.
  void initialize(cv::Mat &image, std::vector<cv::Point> &contours) {
    if (parentNode_ != NULL) {
      RCLCPP_WARN(parentNode_->get_logger(), "Contour %d initializing", contourID_);
    }

    // Convert points.
    for (unsigned int i = 0; i < contours.size() - 1; i++) {
      std::shared_ptr<geometry_msgs::msg::Point> pointMsg = std::make_shared<geometry_msgs::msg::Point>();
      
      
      if (!image.empty()) {
        // Center image.
        pointMsg->x -= image.cols / 2;
        pointMsg->y -= image.rows / 2;

        // Normalise points to range 0-1.
        pointMsg->x /= image.cols;
        pointMsg->y /= image.rows;
      }
      
      points_.push_back(pointMsg);
    }

    selfPtr_ = shared_from_this();
  }
};

#endif // CONTOUR_H