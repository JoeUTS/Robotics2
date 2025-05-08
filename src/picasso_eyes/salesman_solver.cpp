#include "salesman_solver.h"

SalesmanSolver::SalesmanSolver(std::shared_ptr<rclcpp::Node> parentNode) 
  : parentNode_(parentNode) {

}

void SalesmanSolver::setContourList(std::map<int, std::shared_ptr<Contour>> &contourList) {
  contourList_.clear(); // Clear any existing data
  
  /*
  This totally works but work smarter not harder and use caveman brain - Joseph
  for (const auto& [key, value] : contourList) {
    contourList_[key] = std::make_shared<Contour>(key); // Ensure Contour is created with the required argument
  }
  */
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

// void SalesmanSolver::solve() {
//   travelOrder_.clear();

//   if (contourList_.empty()) {
//     if (parentNode_ != NULL) {
//       RCLCPP_WARN(parentNode_->get_logger(), "Contour list is empty.");
//     }
    
//     return;
//   }

//   std::vector<int> contourID;
//   contourID.reserve(contourList_.size());  // Preallocating memory can increase code speed drastically with large vectors - Joseph
//   for (const auto& [key, _] : contourList_) {
//     contourID.push_back(key);
//   }

//   std::vector<int> bestOrder;
//   bestOrder.reserve(contourID.size());
//   double minDistance = std::numeric_limits<double>::max();

//   do { 
//     double totalDistance = 0.0;
//     for (size_t i = 0; i < contourID.size() - 1; ++i) {
//       auto from = contourList_.at(contourID[i])->getHead();
//       auto to = contourList_.at(contourID[i + 1])->getHead();
//       totalDistance += computeDistance(*from, *to);
//     }

//     if (totalDistance < minDistance) {
//       minDistance = totalDistance;
//       bestOrder = contourID;
//     }
//   } while (std::next_permutation(contourID.begin(), contourID.end()));

//   for (int contourID : bestOrder) 
//     bool direction = 
//     travelOrder_.emplace_back(contourID, true); // 'true' indicates the contour is active
//   }

//   if (parentNode_ != NULL) {
//     RCLCPP_INFO(parentNode_->get_logger(), "Total distance: %.2f", minDistance);
//   }
// }
void SalesmanSolver::solve() {
 // RCLCPP_INFO(parentNode_->get_logger(), "Starting TSP solver...");
  std::cout << "Starting TSP solver..." << std::endl << std::flush;
  travelOrder_.clear();

  

  if (contourList_.empty()) {
      if (parentNode_ != NULL or parentNode_ != nullptr) {
          RCLCPP_WARN(parentNode_->get_logger(), "Contour list is empty.");
      }
      std::cout << "Contour list is empty." << std::endl;
      return;
  }

  // Prepare data structures for recursive functions
  std::vector<int> current_contour_order_ids; // Current permutation of contour IDs
  std::map<int, bool> used_ids;               // Tracks which contour IDs have been used
  for (const auto& [key, _] : contourList_) {
      used_ids[key] = false; // Initialize all IDs as unused
  }

  double min_travel_distance = std::numeric_limits<double>::max(); // Minimum travel distance
  std::vector<int> best_contour_order_ids;                         // Best order of contour IDs
  std::vector<bool> best_directions;                               // Best directions for contours

  std::cout << "Initialized data structures for TSP solver." << std::endl;

  // Generate all permutations and direction combinations
  generatePermutations(
      contourList_,               // Original contours
      current_contour_order_ids,  // Current permutation of contour IDs
      used_ids,                   // Tracks used contour IDs
      min_travel_distance,        // Minimum travel distance (updated by the function)
      best_contour_order_ids,     // Best order of contour IDs (updated by the function)
      best_directions             // Best directions for contours (updated by the function)
  );

  std::cout << "Finished generating permutations and directions." << std::endl;
  std::cout << "Best travel distance: " << min_travel_distance << std::endl;
  std::cout << "Best contour order: ";
  for (int id : best_contour_order_ids) {
      std::cout << id << " ";
  }
  std::cout << std::endl;

  std::cout << "Best directions: ";
  for (bool dir : best_directions) {
      std::cout << (dir ? "Forward " : "Backward ");
  }
  std::cout << std::endl;
  // Populate the travelOrder_ with the best order and directions
  for (int contourID : best_contour_order_ids) {
    bool direction = true; // Default to forward
    // If this is the first contour, set the direction to false
    if (contourID == best_contour_order_ids[0]) {
      direction = false;
    }
    travelOrder_.emplace_back(contourID, direction);
  }
  if (parentNode_ != NULL) {
      RCLCPP_INFO(parentNode_->get_logger(), "Total distance: %.2f", min_travel_distance);
  }
}



// Function to calculate the total travel distance for a given path (permutation of contour IDs + directions)
double SalesmanSolver::calculateTotalTravelDistance(
  const std::map<int, std::shared_ptr<Contour>>& original_contours, // Changed to map of shared_ptr<Contour>
  const std::vector<int>& contour_order_ids,    // Stores map keys (contour IDs)
  const std::vector<bool>& directions)  // true = forwards, false = backwards
{
  double total_travel_distance = 0.0;

  // Iterate through the contours in the specified order of IDs
  for (size_t i = 0; i < contour_order_ids.size() - 1; ++i) {
      int current_contour_id = contour_order_ids[i];
      int next_contour_id = contour_order_ids[i + 1];

      // Access contours from the map using their IDs and get the shared_ptr
      std::shared_ptr<Contour> current_contour = original_contours.at(current_contour_id);
      std::shared_ptr<Contour> next_contour = original_contours.at(next_contour_id);

      // Determine the end point of the current contour based on its direction
      geometry_msgs::msg::Point current_contour_end_point;
      if (directions[i] == true) {
          // Dereference the shared_ptr returned by getTail() to get the Point object
          current_contour_end_point = *(current_contour->getTail());
      } else { // Direction::Backward
          // Dereference the shared_ptr returned by getHead() to get the Point object
          current_contour_end_point = *(current_contour->getHead());
      }

      // Determine the start point of the next contour based on its direction
      geometry_msgs::msg::Point next_contour_start_point;
      if (directions[i + 1] == true) {
           // Dereference the shared_ptr returned by getHead() to get the Point object
          next_contour_start_point = *(next_contour->getHead());
      } else { // Direction::Backward
           // Dereference the shared_ptr returned by getTail() to get the Point object
          next_contour_start_point = *(next_contour->getTail());
      }

      // Add the distance between the end of the current contour and the start of the next contour
      total_travel_distance += computeDistance(current_contour_end_point, next_contour_start_point);
  }

  return total_travel_distance;
}

// Recursive function to generate all combinations of directions for a given contour order (of IDs)
void SalesmanSolver::generateDirectionCombinations(
  const std::map<int, std::shared_ptr<Contour>>& original_contours, // Changed to map of shared_ptr<Contour>
  const std::vector<int>& contour_order_ids,    // Stores map keys (contour IDs)
  std::vector<bool>& current_directions,
  size_t k, // Current contour index in the order being considered (0 to num_contours-1)
  double& min_travel_distance,
  std::vector<int>& best_contour_order_ids, // Stores map keys (contour IDs)
  std::vector<bool>& best_directions)
{
  // Base case: All contours in the order have been assigned a direction
  if (k == original_contours.size()) {
      // Calculate the travel distance for this specific path (order + directions)
      double current_travel_distance = calculateTotalTravelDistance(
          original_contours,
          contour_order_ids,
          current_directions);
    
       //   std::cout << "Evaluating directions: ";
     //   for (bool dir : current_directions) {
     //       std::cout << (dir ? "Forward " : "Backward ");
     //   }
      //  std::cout << "| Distance: " << current_travel_distance << std::endl;

      // Check if this path is better than the current best
      if (current_travel_distance < min_travel_distance) {
          min_travel_distance = current_travel_distance;
          best_contour_order_ids = contour_order_ids;
          best_directions = current_directions;
          std::cout << "New best path found with distance: " << min_travel_distance << std::endl;
      }
      return;
  }

  // Recursive step: Try both directions for the current contour at index k in the order
  // Try Forward
  current_directions[k] = true;
  generateDirectionCombinations(
      original_contours,
      contour_order_ids,
      current_directions,
      k + 1,
      min_travel_distance,
      best_contour_order_ids,
      best_directions);

  // Try Backward
  current_directions[k] = false;
  generateDirectionCombinations(
      original_contours,
      contour_order_ids,
      current_directions,
      k + 1,
      min_travel_distance,
      best_contour_order_ids,
      best_directions);
}

// Recursive function to generate all permutations of contour IDs
void SalesmanSolver::generatePermutations(
  const std::map<int, std::shared_ptr<Contour>>& original_contours, // Changed to map of shared_ptr<Contour>
  std::vector<int>& current_contour_order_ids, // Stores map keys (contour IDs)
  std::map<int, bool>& used_ids,            // Tracks which contour IDs have been used
  double& min_travel_distance,
  std::vector<int>& best_contour_order_ids,    // Stores map keys (contour IDs)
  std::vector<bool>& best_directions)
{
  // Base case: A complete permutation of contour IDs has been formed
  if (current_contour_order_ids.size() == original_contours.size()) {
      // Now generate all direction combinations for this permutation of IDs
      std::vector<bool> current_directions(original_contours.size());
      generateDirectionCombinations(
          original_contours,
          current_contour_order_ids,
          current_directions,
          0, // Start generating directions from the first contour in the order
          min_travel_distance,
          best_contour_order_ids,
          best_directions);
      return;
  }

  // Recursive step: Try adding each unused contour ID to the current order
  // Iterate through the map keys (contour IDs)
  for (auto const& [contour_id, contour_ptr] : original_contours) {
      if (!used_ids[contour_id]) {
          used_ids[contour_id] = true;
          current_contour_order_ids.push_back(contour_id); // Add contour ID to the current order

          generatePermutations(
              original_contours,
              current_contour_order_ids,
              used_ids,
              min_travel_distance,
              best_contour_order_ids,
              best_directions);

          // Backtrack: Remove the contour ID and mark it as unused for the next iteration
          current_contour_order_ids.pop_back();
          used_ids[contour_id] = false;
      }
  }
}