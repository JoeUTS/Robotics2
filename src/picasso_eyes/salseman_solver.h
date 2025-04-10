#ifndef SALSEMANSOLVER_H
#define SALSEMANSOLVER_H

#include <map>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "Contour.h"

class SalsemanSolver : public rclcpp::Node {
public:
  SalsemanSolver(void);

  void setContourList(std::map<int, std::shared_ptr<Contour>> contourList);

  std::vector<std::pair<int, bool>> getTravelOrder(void);

  private:
  std::map<int, std::shared_ptr<Contour>> contourList_; // List of contours.

  std::vector<std::pair<int, bool>> travelOrder_;     // List of travel order.
};

#endif // SALSEMANSOLVER_H