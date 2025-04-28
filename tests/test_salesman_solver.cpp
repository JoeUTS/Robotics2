#include "../src/picasso_eyes/salesman_solver.h"
#include <gtest/gtest.h>
#include <memory>

TEST(SalesmanSolverTest, SetAndGetTravelOrder) {

  int argc = 0;
  char** argv = nullptr;
  rclcpp::init(argc, argv);

  try{
  // Create an instance of SalesmanSolver
  SalesmanSolver solver(NULL);

  // Create a mock contour list - More than two points needed to test - Joseph
  cv::Mat emptyImage;
  std::vector<cv::Point> contour1;
  contour1.push_back(cv::Point(0, 0));
  contour1.push_back(cv::Point(1, 1));
  std::vector<cv::Point> contour2;
  contour2.push_back(cv::Point(2, 2));
  contour2.push_back(cv::Point(3, 3));
  std::vector<cv::Point> contour3;
  contour3.push_back(cv::Point(4, 4));
  contour3.push_back(cv::Point(5, 5));
  std::vector<cv::Point> contour4;
  contour4.push_back(cv::Point(6, 6));
  contour4.push_back(cv::Point(7, 7));
  std::vector<cv::Point> contour5;
  contour5.push_back(cv::Point(8, 8));
  contour5.push_back(cv::Point(9, 9));

  std::map<int, std::shared_ptr<Contour>> contourList;
  contourList.insert(std::pair<int, std::shared_ptr<Contour>>(3, Contour::create(3, contour3, emptyImage)));
  contourList.insert(std::pair<int, std::shared_ptr<Contour>>(1, Contour::create(1, contour1, emptyImage)));
  contourList.insert(std::pair<int, std::shared_ptr<Contour>>(5, Contour::create(5, contour5, emptyImage)));
  contourList.insert(std::pair<int, std::shared_ptr<Contour>>(4, Contour::create(4, contour4, emptyImage)));
  contourList.insert(std::pair<int, std::shared_ptr<Contour>>(2, Contour::create(2, contour2, emptyImage)));

  // Test function
  // Original function has the test setting a vector and testing if the vector was equal to what you had just set.
  // solve() needs to be called to test the function accurately. - Joseph
  solver.setContourList(contourList);
  solver.solve();
  auto result = solver.getTravelOrder();

  // Verify the travel order
  ASSERT_EQ(result.size(), 5);
  EXPECT_EQ(result[0].first, 1);
  EXPECT_EQ(result[0].second, false);
  EXPECT_EQ(result[1].first, 2);
  EXPECT_EQ(result[1].second, true);
  EXPECT_EQ(result[2].first, 3);
  EXPECT_EQ(result[2].second, true);
  EXPECT_EQ(result[3].first, 4);
  EXPECT_EQ(result[3].second, true);
  EXPECT_EQ(result[4].first, 5);
  EXPECT_EQ(result[4].second, true);

}
  catch (const std::exception &e) {
    std::cerr << "Exception: " << e.what() << std::endl;
  }

  rclcpp::shutdown();
}