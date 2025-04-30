#include "../src/picasso_eyes/salesman_solver.h"
#include <opencv2/opencv.hpp>
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
    
    std::vector<std::shared_ptr<geometry_msgs::msg::Point>> contour1Points;
    std::shared_ptr<geometry_msgs::msg::Point> point1 = std::make_shared<geometry_msgs::msg::Point>();
    point1 -> x = 0;
    point1 -> y = 0;
    contour1Points.push_back(point1);
    std::shared_ptr<geometry_msgs::msg::Point> point2 = std::make_shared<geometry_msgs::msg::Point>();
    point2 -> x = 1;
    point2 -> y = 1;
    contour1Points.push_back(point2);
    std::shared_ptr<Contour> contour1 = std::make_shared<Contour>(1, contour1Points);

    std::vector<std::shared_ptr<geometry_msgs::msg::Point>> contour2Points;
    std::shared_ptr<geometry_msgs::msg::Point> point3 = std::make_shared<geometry_msgs::msg::Point>();
    point3 -> x = 2;
    point3 -> y = 2;
    contour2Points.push_back(point3);
    std::shared_ptr<geometry_msgs::msg::Point> point4 = std::make_shared<geometry_msgs::msg::Point>();
    point4 -> x = 3;
    point4 -> y = 3;
    contour2Points.push_back(point4);
    std::shared_ptr<Contour>  contour2 = std::make_shared<Contour>(2, contour2Points);

    std::vector<std::shared_ptr<geometry_msgs::msg::Point>> contour3Points;
    std::shared_ptr<geometry_msgs::msg::Point> point5 = std::make_shared<geometry_msgs::msg::Point>();
    point5 -> x = 4;
    point5 -> y = 4;
    contour3Points.push_back(point5);
    std::shared_ptr<geometry_msgs::msg::Point> point6 = std::make_shared<geometry_msgs::msg::Point>();
    point6 -> x = 5;
    point6 -> y = 5;
    contour3Points.push_back(point6);
    std::shared_ptr<Contour>  contour3 = std::make_shared<Contour>(3, contour3Points);

    std::vector<std::shared_ptr<geometry_msgs::msg::Point>> contour4Points;
    std::shared_ptr<geometry_msgs::msg::Point> point7 = std::make_shared<geometry_msgs::msg::Point>();
    point7 -> x = 6;
    point7 -> y = 6;
    contour4Points.push_back(point7);
    std::shared_ptr<geometry_msgs::msg::Point> point8 = std::make_shared<geometry_msgs::msg::Point>();
    point8 -> x = 7;
    point8 -> y = 7;
    contour4Points.push_back(point8);
    std::shared_ptr<Contour>  contour4 = std::make_shared<Contour>(4, contour4Points);

    std::vector<std::shared_ptr<geometry_msgs::msg::Point>> contour5Points;
    std::shared_ptr<geometry_msgs::msg::Point> point9 = std::make_shared<geometry_msgs::msg::Point>();
    point9 -> x = 8;
    point9 -> y = 8;
    contour5Points.push_back(point9);
    std::shared_ptr<geometry_msgs::msg::Point> point10 = std::make_shared<geometry_msgs::msg::Point>();
    point10 -> x = 9;
    point10 -> y = 9;
    contour5Points.push_back(point10);
    std::shared_ptr<Contour>  contour5 = std::make_shared<Contour>(5, contour5Points);

    void setContourList(std::map<int, std::shared_ptr<Contour>> &contourList); 
    std::map<int, std::shared_ptr<Contour>> contourList;
    contourList.insert(std::pair<int, std::shared_ptr<Contour>>(3, contour3));
    contourList.insert(std::pair<int, std::shared_ptr<Contour>>(1, contour1));
    contourList.insert(std::pair<int, std::shared_ptr<Contour>>(5, contour5));
    contourList.insert(std::pair<int, std::shared_ptr<Contour>>(4, contour4));
    contourList.insert(std::pair<int, std::shared_ptr<Contour>>(2, contour2));

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