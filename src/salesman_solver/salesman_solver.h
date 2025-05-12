#ifndef SALESMANSOLVER_H
#define SALESMANSOLVER_H

#include <map>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "../common/Contour.h"

class SalesmanSolver { // doesnt need to be its own node - Joseph
public:
  SalesmanSolver(std::shared_ptr<rclcpp::Node> parentNode = NULL);

  void setContourList(std::map<int, std::shared_ptr<Contour>> &contourList);  // using '&' to pass contourList by reference instead of copy (less memory used) - Joseph

  std::vector<std::pair<int, bool>> getTravelOrder(void);

  void setTravelOrder(std::vector<std::pair<int, bool>> &travelOrder); // As above - Joseph

  void solve();

  

  private:
  std::shared_ptr<rclcpp::Node> parentNode_;  // Parent node pointer. We will access the logger via this pointer.
  std::map<int, std::shared_ptr<Contour>> contourList_; // List of contours.

  std::vector<std::pair<int, bool>> travelOrder_;     // List of travel order.

  double computeDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) const;




  // Joseph's AI generated code. Calculates all permutations of contour IDs. - Not tested yet
  void generatePermutations(const std::map<int, std::shared_ptr<Contour>>& original_contours, 
                                            std::vector<int>& current_contour_order_ids, std::map<int, bool>& used_ids,
                                            double& min_travel_distance, std::vector<int>& best_contour_order_ids, 
                                            std::vector<bool>& best_directions);
  
  void generateDirectionCombinations(const std::map<int, std::shared_ptr<Contour>>& original_contours, 
                                    const std::vector<int>& contour_order_ids, std::vector<bool>& current_directions,
                                    size_t k, double& min_travel_distance,
                                    std::vector<int>& best_contour_order_ids, std::vector<bool>& best_directions);

  double calculateTotalTravelDistance(const std::map<int, std::shared_ptr<Contour>>& original_contours,  const std::vector<int>& contour_order_ids,
                                                      const std::vector<bool>& directions);
};

#endif // SALESMANSOLVER_H