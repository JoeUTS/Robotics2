#ifndef SALESMANSOLVER_H
#define SALESMANSOLVER_H

#include <map>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "Contour.h"

class SalesmanSolver : public rclcpp::Node {
public:
  SalesmanSolver(void);

  void setContourList(std::map<int, std::shared_ptr<Contour>> contourList);

  std::vector<std::pair<int, bool>> getTravelOrder(void);

  void setTravelOrder(std::vector<std::pair<int, bool>> travelOrder);

  void solve();

  private:
  std::map<int, std::shared_ptr<Contour>> contourList_; // List of contours.

  std::vector<std::pair<int, bool>> travelOrder_;     // List of travel order.

  double computeDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) const;
};

#endif // SALESMANSOLVER_H