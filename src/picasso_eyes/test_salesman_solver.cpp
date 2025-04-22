#include "salesman_solver.h"
#include <gtest/gtest.h>
#include <memory>

TEST(SalesmanSolverTest, SetAndGetTravelOrder) {

  int argc = 0;
  char** argv = nullptr;
  rclcpp::init(argc, argv);

  try{
  // Create an instance of SalesmanSolver
  SalesmanSolver solver;

  // Create a mock contour list
  std::map<int, std::shared_ptr<Contour>> contourList;
  contourList[1] = std::make_shared<Contour>(1);
  contourList[2] = std::make_shared<Contour>(2);

  // Set the contour list
  solver.setContourList(contourList);

  // Mock travel order
std::vector<std::pair<int, bool>> travelOrder = {{1, false}, {2, true}};
solver.setTravelOrder(travelOrder); // Use the public setter method
  // Get the travel order
  auto result = solver.getTravelOrder();

  // Verify the travel order
  ASSERT_EQ(result.size(), 2);
  EXPECT_EQ(result[0].first, 1);
  EXPECT_EQ(result[0].second, false);
  EXPECT_EQ(result[1].first, 2);
  EXPECT_EQ(result[1].second, true);
}
  catch (const std::exception &e) {
    std::cerr << "Exception: " << e.what() << std::endl;
  }

  rclcpp::shutdown();
}