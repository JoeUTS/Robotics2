#ifndef SALESMAN_SOLVER_H
#define SALESMAN_SOLVER_H

#include <map>
#include <memory>
#include <chrono>
#include <random>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "../common/Contour.h"

struct TspNode {
  int contourID;
  bool direction; // True if traversing forward
};

class SalesmanSolver {
public:
  SalesmanSolver(std::shared_ptr<rclcpp::Node> parentNode = NULL);

  void setContourList(std::map<int, std::shared_ptr<Contour>> &contourList);

  std::vector<std::pair<int, bool>> getTravelOrder(void);

  void setTravelOrder(std::vector<std::pair<int, bool>> &travelOrder);

  void solve();

  void solveTsp2OptWithOrientations(void);

private:
  std::shared_ptr<rclcpp::Node> parentNode_;  // Parent node pointer for logging.
  std::map<int, std::shared_ptr<Contour>> contourList_; // List of contours.
  std::vector<std::pair<int, bool>> travelOrder_;     // List of travel order.
  unsigned long long permutationTotal_;
  unsigned long long permutationCurrent_;

  /// @brief Calculate distance between two points.
  /// @param a Point A
  /// @param b point B
  /// @return Distance between points
  double computeDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) const;
  
  /// @brief Recursive function to generate all permutations of contour IDs
  /// @param contours map of contours
  /// @param contourOrder Contour ID vector
  /// @param idUsed Used IDs
  /// @param minDistance Minimum travel distance
  /// @param orderBest Best contour order (of IDs)
  /// @param directionBest Best directions
  void generatePermutations(const std::map<int, std::shared_ptr<Contour>>& contours,
                            std::vector<int>& contourOrder,
                            std::map<int, bool>& idUsed,
                            double& minDistance,
                            std::vector<int>& orderBest,
                            std::vector<bool>& directionBest);
  
  /// @brief Recursive function to generate all combinations of directions for a given contour order (of IDs)
  /// @param contours map of contours
  /// @param contourID Contour ID vector
  /// @param directions [true = forwards, false = backwards] Contour draw direction
  /// @param contourIndex Index of current contour
  /// @param distanceMin Shortest distance
  /// @param orderBest Best order vector
  /// @param directionBest Best directions vector
  void generateDirectionCombinations(const std::map<int, std::shared_ptr<Contour>>& contours,
                                    const std::vector<int>& contourID,
                                    std::vector<bool>& directions,
                                    size_t contourIndex,
                                    double& distanceMin,
                                    std::vector<int>& orderBest,
                                    std::vector<bool>& directionBest);
  
  /// @brief Calculate total travel distance
  /// @param contours map of contours
  /// @param contourOrder Contour order vector to calculate distance of
  /// @param directions Directions vector
  /// @return Total distance
  double calculateTotalTravelDistance(const std::map<int, std::shared_ptr<Contour>>& contours,
                                      const std::vector<int>& contourOrder,
                                      const std::vector<bool>& directions);


  std::shared_ptr<geometry_msgs::msg::Point> getContourStart(const std::shared_ptr<Contour>& contour,
                                                          bool startsFromFirstPoint);

  std::shared_ptr<geometry_msgs::msg::Point> getContourEnd(const std::shared_ptr<Contour>& contour, 
                                                        bool startsFromFirstPoint);

  double calculateTourDistance(const std::vector<int>& tourIndices,
                             const std::vector<std::vector<double>>& distanceMatrix);

  
};

#endif // SALESMAN_SOLVER_H
