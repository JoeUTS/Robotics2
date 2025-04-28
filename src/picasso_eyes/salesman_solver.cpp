#include "salesman_solver.h"

SalesmanSolver::SalesmanSolver(void)
  : rclcpp::Node("salesman_solver") {

}

void SalesmanSolver::setContourList(std::map<int, std::shared_ptr<Contour>> contourList) {
  contourList_.clear(); // Clear any existing data
  for (const auto& [key, value] : contourList) {
    contourList_[key] = std::make_shared<Contour>(key); // Ensure Contour is created with the required argument
  }
}

std::vector<std::pair<int, bool>> SalesmanSolver::getTravelOrder(void) {
  return travelOrder_;
}

void SalesmanSolver::setTravelOrder(std::vector<std::pair<int, bool>> travelOrder) {
  travelOrder_ = travelOrder;
}

double SalesmanSolver::computeDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) const {
  return std::hypot(b.x - a.x, b.y - a.y);
}

void SalesmanSolver::solve() {
  travelOrder_.clear();

  if (contourList_.empty()) {
    RCLCPP_WARN(this->get_logger(), "Contour list is empty");
    return;
  }

  std::vector<int> keys; // Vector to hold the keys of the contour list, which are the contour IDs
  for (const auto& [key, _] : contourList_) {
    keys.push_back(key); 
  }

  std::vector<int> bestOrder;
  double minDistance = std::numeric_limits<double>::max(); 

  do {
    double totalDistance = 0.0;
    for (size_t i = 0; i < keys.size() - 1; ++i) { // Loop through the keys
      auto from = contourList_.at(keys[i])->getHead(); // Get the head of the contour
      auto to = contourList_.at(keys[i + 1])->getHead(); // Get the head of the next contour
      totalDistance += computeDistance(*from, *to); // Compute distance between the two points
    }

    if (totalDistance < minDistance) {
      minDistance = totalDistance;
      bestOrder = keys;
    }
  } while (std::next_permutation(keys.begin(), keys.end()));

  for (int key : bestOrder) {
    travelOrder_.emplace_back(key, true); // 'true' indicates the contour is active
  }

  RCLCPP_INFO(this->get_logger(), "Total distance= %.2f", minDistance);
}