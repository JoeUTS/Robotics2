#ifndef CONTOUR_H
#define CONTOUR_H

#include <vector>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point.hpp>

class Contour : public std::enable_shared_from_this<Contour> {
public:
  /// @brief Constructor, sets ID and points vector.
  /// @param contourID ID of contour.
  Contour(const int contourID, std::vector<std::shared_ptr<geometry_msgs::msg::Point>> contourPoints);

  /// @brief ID getter.
  /// @return int. contourID_.
  int getID(void);

  /// @brief self pointer getter.
  /// @return std::shared_ptr<Contour>. selfPtr_.
  std::shared_ptr<Contour> getContourPtr(void);

  /// @brief is drawn getter.
  /// @return bool. isDrawn_.
  bool getDrawn(void);

  /// @brief Drawn status setter true.
  void setDrawn(bool state);

  /// @brief first point getter.
  /// @return std::shared_ptr<geometry_msgs::msg::Point>. points_.front().
  std::shared_ptr<geometry_msgs::msg::Point> getHead(void);

  /// @brief last point getter.
  /// @return std::shared_ptr<geometry_msgs::msg::Point>. points_.back().
  std::shared_ptr<geometry_msgs::msg::Point> getTail(void);

  /// @brief Gets points list from start to end as a PoseArray.
  /// @return geometry_msgs::msg::PoseArray. points_.
  geometry_msgs::msg::PoseArray getPath(void);

  /// @brief Gets points list from end to start as a PoseArray.
  /// @return geometry_msgs::msg::PoseArray. points_.
  geometry_msgs::msg::PoseArray getPathBackwards(void);

  /// @brief Get list of points as a vector of points.
  /// @return std::vector<geometry_msgs::msg::Point>. points_.
  std::vector<geometry_msgs::msg::Point> getPoints(void);

private:
  const unsigned int contourID_;  // Contour ID.
  const std::vector<std::shared_ptr<geometry_msgs::msg::Point>> contourPoints_;  // Contour's points.
  
  bool isDrawn_ = false;          // True if contour is complete.
};

#endif // CONTOUR_H