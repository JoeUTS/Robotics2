#include "salesman_solver.h"

SalesmanSolver::SalesmanSolver(std::shared_ptr<rclcpp::Node> parentNode) 
  : parentNode_(parentNode) {

}

void SalesmanSolver::setContourList(std::map<int, std::shared_ptr<Contour>> &contourList) {
  contourList_.clear();
  contourList_ = contourList;
}

std::vector<std::pair<int, bool>> SalesmanSolver::getTravelOrder(void) {
  return travelOrder_;
}

void SalesmanSolver::setTravelOrder(std::vector<std::pair<int, bool>> &travelOrder) {
  travelOrder_ = travelOrder;
}

double SalesmanSolver::computeDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) const {
  return std::hypot(b.x - a.x, b.y - a.y);
}

void SalesmanSolver::solve() {
  RCLCPP_INFO(parentNode_->get_logger(), "Starting TSP solver...");
  travelOrder_.clear();

  if (contourList_.empty()) {
      RCLCPP_WARN(parentNode_->get_logger(), "Cannot solve, contour list is empty.");
      return;
  }

  // Prepare data structures for recursive functions
  permutationCurrent_ = 0;
  permutationTotal_ = 1;
  for (size_t i = 2; i <= contourList_.size(); ++i) {
    permutationTotal_ *= i;
  }

  std::vector<int> contourOrder;
  std::map<int, bool> idUsed;
  for (const auto& [key, _] : contourList_) {
    idUsed[key] = false; 
  }

  double distanceMin = std::numeric_limits<double>::max();
  std::vector<int> orderBest;
  std::vector<bool> directionBest;

  // Solve via brute force.
  generatePermutations(
    contourList_,
    contourOrder,
    idUsed,
    distanceMin,
    orderBest,
    directionBest
  );

  // Update travel order
  for (int contourID : orderBest) {
    bool direction = true;

    if (contourID == orderBest[0]) {
      direction = false;
    }
    travelOrder_.emplace_back(contourID, direction);
  }
}

double SalesmanSolver::calculateTotalTravelDistance(
  const std::map<int, std::shared_ptr<Contour>>& contours,
  const std::vector<int>& contourOrder,
  const std::vector<bool>& directions)  // true = forwards, false = backwards
{
  double distance = 0.0;

  for (size_t i = 0; i < contourOrder.size() - 1; ++i) {
    std::shared_ptr<Contour> contourCurrent = contours.at(contourOrder[i]);
    std::shared_ptr<Contour> contourNext = contours.at(contourOrder[i + 1]);

    geometry_msgs::msg::Point pointStart;
    if (directions[i] == true) {  // Forward
      pointStart = *(contourCurrent->getTail());

    } else { // Backward
      pointStart = *(contourCurrent->getHead());
    }

    geometry_msgs::msg::Point pointEnd;
    if (directions[i + 1] == true) {  // Forward
      pointEnd = *(contourNext->getHead());

    } else { // Backward
      pointEnd = *(contourNext->getTail());
    }

    distance += computeDistance(pointStart, pointEnd);
  }

  return distance;
}

void SalesmanSolver::generateDirectionCombinations(
  const std::map<int, std::shared_ptr<Contour>>& contours,
  const std::vector<int>& contourID,
  std::vector<bool>& directions,
  size_t contourIndex,
  double& distanceMin,
  std::vector<int>& orderBest,
  std::vector<bool>& directionBest) {

  // Orders generated
  if (contourIndex == contours.size()) {
    RCLCPP_INFO(parentNode_->get_logger(), "Calculation step [%llu/%llu]", ++permutationCurrent_, permutationTotal_);

    double current_travel_distance = calculateTotalTravelDistance(contours,
                                                                  contourID,
                                                                  directions);

    if (current_travel_distance < distanceMin) {
      distanceMin = current_travel_distance;
      orderBest = contourID;
      directionBest = directions;
    }

    return;
  }

  // recursively generatate for both permutations
  directions[contourIndex] = true;
  generateDirectionCombinations(
    contours,
    contourID,
    directions,
    contourIndex + 1,
    distanceMin,
    orderBest,
    directionBest);

  directions[contourIndex] = false;
  generateDirectionCombinations(
    contours,
    contourID,
    directions,
    contourIndex + 1,
    distanceMin,
    orderBest,
    directionBest);
}

void SalesmanSolver::generatePermutations(
  const std::map<int, std::shared_ptr<Contour>>& contours,
  std::vector<int>& contourOrder,
  std::map<int, bool>& idUsed,
  double& minDistance,
  std::vector<int>& orderBest,
  std::vector<bool>& directionBest)
{
  // ID list has been filled
  if (contourOrder.size() == contours.size()) {
    std::vector<bool> directions(contours.size());
    generateDirectionCombinations(
      contours,
      contourOrder,
      directions,
      0, // start ID
      minDistance,
      orderBest,
      directionBest);
    return;
  }

  RCLCPP_INFO(parentNode_->get_logger(), "Permutation step [%zu/%zu]", contourOrder.size(), contours.size());
  
  // ID list is not yet filled
  for (auto const& [contour_id, contour_ptr] : contours) {
    if (!idUsed[contour_id]) {
      idUsed[contour_id] = true;
      contourOrder.push_back(contour_id); // Add contour ID to the current order

      generatePermutations(
        contours,
        contourOrder,
        idUsed,
        minDistance,
        orderBest,
        directionBest);

      // reset for next iteration
      contourOrder.pop_back();
      idUsed[contour_id] = false;
    }
  }
}


std::shared_ptr<geometry_msgs::msg::Point> SalesmanSolver::getContourStart(const std::shared_ptr<Contour>& contour,
                                                          bool startsFromFirstPoint) {
  if (startsFromFirstPoint) {
    return contour->getHead();
  } else {
    return contour->getTail();
  }
}

std::shared_ptr<geometry_msgs::msg::Point> SalesmanSolver::getContourEnd(const std::shared_ptr<Contour>& contour, 
                                                        bool startsFromFirstPoint) {
  if (startsFromFirstPoint) {
    return contour->getTail();
  } else {
    return contour->getHead();
  }
}

double SalesmanSolver::calculateTourDistance(const std::vector<int>& tourIndices,
                             const std::vector<std::vector<double>>& distanceMatrix) {
  double totalDistance = 0.0;

  if (tourIndices.empty()) {

    return 0.0;
  }

  for (size_t i = 0; i < tourIndices.size(); ++i) {
    int currentNodeIdx = tourIndices[i];
    int nextNodeIdx = tourIndices[(i + 1) % tourIndices.size()]; // Loop back to start

    totalDistance += distanceMatrix[currentNodeIdx][nextNodeIdx];
  }

  return totalDistance;
}

// Implements the 2-Opt local search algorithm for TSP with orientations
void SalesmanSolver::solveTsp2OptWithOrientations(void) {
  if (contourList_.empty()) {
    RCLCPP_INFO(parentNode_->get_logger(), "Empty contour list.");
    return;
  }

  travelOrder_.clear();

  std::vector<TspNode> tspNodes;
  tspNodes.reserve(contourList_.size() * 2);

  for (const auto& pair : contourList_) {
    int contourID = pair.first;
    tspNodes.push_back({contourID, true}); // Forward node
    tspNodes.push_back({contourID, false}); // Backward node
  }

  int numOriginalContours = contourList_.size();
  int numTspNodes = tspNodes.size();

  // Precompute distance matrix
  std::vector<std::vector<double>> distanceMatrix(numTspNodes, std::vector<double>(numTspNodes));

  for (int i = 0; i < numTspNodes; ++i) {
    for (int j = 0; j < numTspNodes; ++j) {
      const TspNode& node1 = tspNodes[i];
      const TspNode& node2 = tspNodes[j];

      if (i == j) {
        distanceMatrix[i][j] = 0.0;
        continue;
      }

      if (node1.contourID == node2.contourID) {
        distanceMatrix[i][j] = std::numeric_limits<double>::max(); // Block direct transitions within same contour
        continue;
      }

      geometry_msgs::msg::Point endPointC1 = *getContourEnd(contourList_.at(node1.contourID), 
                                                          node1.direction);
      geometry_msgs::msg::Point startPointC2 = *getContourStart(contourList_.at(node2.contourID), 
                                                              node2.direction);

      distanceMatrix[i][j] = computeDistance(endPointC1, startPointC2);
    }
  }

  // Generate random initial tour
  std::vector<int> currentTourIndices;
  std::vector<bool> visitedOriginalContour(numOriginalContours, false); // Tracks if original contour has been selected
  std::random_device rd;
  std::mt19937 g(rd());
  std::uniform_int_distribution<> originalContourDist(0, numOriginalContours - 1);
  std::uniform_int_distribution<> orientationDist(0, 1);

  int firstOriginalContourId = originalContourDist(g);
  bool firstContourOrientation = (orientationDist(g) == 0);
  int firstTspNodeIndex = firstOriginalContourId * 2 + (firstContourOrientation ? 0 : 1);

  currentTourIndices.reserve(numOriginalContours);
  currentTourIndices.push_back(firstTspNodeIndex);
  visitedOriginalContour[firstOriginalContourId] = true;

  // Greedily add remaining contours
  while (currentTourIndices.size() < numOriginalContours) {
    int lastNodeInTourIndex = currentTourIndices.back();
    double minDistance = std::numeric_limits<double>::max();
    int bestNextNodeIndex = -1;

    for (int i = 0; i < numTspNodes; ++i) {
      const TspNode& candidateNode = tspNodes[i];
      if (!visitedOriginalContour[candidateNode.contourID]) {
        double dist = distanceMatrix[lastNodeInTourIndex][i];
        if (dist < minDistance) {
          minDistance = dist;
          bestNextNodeIndex = i;
        }
      }
    }

    if (bestNextNodeIndex != -1) {
      currentTourIndices.push_back(bestNextNodeIndex);
      visitedOriginalContour[tspNodes[bestNextNodeIndex].contourID] = true;
    } else {
      break;
    }
  }

  double currentTourDistance = calculateTourDistance(currentTourIndices, distanceMatrix);
  bool improvement = true;

  // Perform 2-Opt local search
  std::vector<int> bestTourIndices = currentTourIndices;
  double bestTourDistance = currentTourDistance;

  int maxIterationsWithoutImprovement = 1000; // Prevent infinite loops
  int iterations = 0;

  while (improvement && iterations < maxIterationsWithoutImprovement) {
    improvement = false;
    for (int i = 0; i < numOriginalContours - 1; ++i) {
      for (int k = i + 1; k < numOriginalContours; ++k) {
        std::vector<int> newTourIndices = currentTourIndices;
        std::reverse(newTourIndices.begin() + i, newTourIndices.begin() + k + 1);

        double newTourDistance = calculateTourDistance(newTourIndices, distanceMatrix);

        if (newTourDistance < currentTourDistance) {
            currentTourIndices = newTourIndices;
            currentTourDistance = newTourDistance;
            improvement = true;
            iterations = 0;
        }
      }
    }
    iterations++;
  }

  // Set travel order
  for (unsigned int i = 0; i < bestTourIndices.size(); i++) {
    travelOrder_.push_back({bestTourIndices[i], tspNodes[bestTourIndices[i]].direction});
  }
}