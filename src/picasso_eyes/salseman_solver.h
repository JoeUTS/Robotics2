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
  SalsemanSolver(std::map<int, std::shared_ptr<Contour>> contourList);

  private:
  std::map<int, std::shared_ptr<Contour>> contourList_;
};

#endif // SALSEMANSOLVER_H