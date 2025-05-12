#include "../src/salesman_solver/salesman_solver.h"
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

    std::cout << "Added contour1 with points: (" << point1->x << ", " << point1->y << "), ("
    << point2->x << ", " << point2->y << ")" << std::endl;


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

    std::cout << "Added contour2 with points: (" << point3->x << ", " << point3->y << "), ("
    << point4->x << ", " << point4->y << ")" << std::endl;


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

    std::cout << "Added contour3 with points: (" << point5->x << ", " << point5->y << "), ("
    << point6->x << ", " << point6->y << ")" << std::endl;


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

    std::cout << "Added contour4 with points: (" << point7->x << ", " << point7->y << "), ("
    << point8->x << ", " << point8->y << ")" << std::endl;


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

    std::cout << "Added contour5 with points: (" << point9->x << ", " << point9->y << "), ("
    << point10->x << ", " << point10->y << ")" << std::endl;


    void setContourList(std::map<int, std::shared_ptr<Contour>> &contourList); 
    std::map<int, std::shared_ptr<Contour>> contourList;
    contourList.insert(std::pair<int, std::shared_ptr<Contour>>(3, contour3));
    contourList.insert(std::pair<int, std::shared_ptr<Contour>>(1, contour1));
    contourList.insert(std::pair<int, std::shared_ptr<Contour>>(5, contour5));
    contourList.insert(std::pair<int, std::shared_ptr<Contour>>(4, contour4));
    contourList.insert(std::pair<int, std::shared_ptr<Contour>>(2, contour2));

    std::cout << "Contour list set with " << contourList.size() << " contours." << std::endl;

    // Test function
    // Original function has the test setting a vector and testing if the vector was equal to what you had just set.
    // solve() needs to be called to test the function accurately. - Joseph
    solver.setContourList(contourList);
    solver.solve();
    auto result = solver.getTravelOrder();

    std::cout << "Travel order size = " << result.size() << std::endl;

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

TEST(SalesmanSolverTest, AlternateTravelOrderTest) {

  int argc = 0;
  char** argv = nullptr;
  rclcpp::init(argc, argv);

  try{
    // Create an instance of SalesmanSolver
    SalesmanSolver solver(NULL);

    // Create a different mock contour list
    std::vector<std::shared_ptr<geometry_msgs::msg::Point>> contour1Points;
    auto a1 = std::make_shared<geometry_msgs::msg::Point>();
    a1->x = 10; a1->y = 10;
    contour1Points.push_back(a1);
    auto a2 = std::make_shared<geometry_msgs::msg::Point>();
    a2->x = 8; a2->y = 8;
    contour1Points.push_back(a2);
    auto contour1 = std::make_shared<Contour>(10, contour1Points);

    std::vector<std::shared_ptr<geometry_msgs::msg::Point>> contour2Points;
    auto b1 = std::make_shared<geometry_msgs::msg::Point>();
    b1->x = -2; b1->y = -1;
    contour2Points.push_back(b1);
    auto b2 = std::make_shared<geometry_msgs::msg::Point>();
    b2->x = -4; b2->y = -3;
    contour2Points.push_back(b2);
    auto contour2 = std::make_shared<Contour>(20, contour2Points);

    std::vector<std::shared_ptr<geometry_msgs::msg::Point>> contour3Points;
    auto c1 = std::make_shared<geometry_msgs::msg::Point>();
    c1->x = 0; c1->y = 5;
    contour3Points.push_back(c1);
    auto c2 = std::make_shared<geometry_msgs::msg::Point>();
    c2->x = 1; c2->y = 3;
    contour3Points.push_back(c2);
    auto contour3 = std::make_shared<Contour>(30, contour3Points);

    std::vector<std::shared_ptr<geometry_msgs::msg::Point>> contour4Points;
    auto d1 = std::make_shared<geometry_msgs::msg::Point>();
    d1->x = 3; d1->y = -5;
    contour4Points.push_back(d1);
    auto d2 = std::make_shared<geometry_msgs::msg::Point>();
    d2->x = 2; d2->y = -4;
    contour4Points.push_back(d2);
    auto contour4 = std::make_shared<Contour>(40, contour4Points);

    std::vector<std::shared_ptr<geometry_msgs::msg::Point>> contour5Points;
    auto e1 = std::make_shared<geometry_msgs::msg::Point>();
    e1->x = 6; e1->y = -2;
    contour5Points.push_back(e1);
    auto e2 = std::make_shared<geometry_msgs::msg::Point>();
    e2->x = 5; e2->y = -1;
    contour5Points.push_back(e2);
    auto contour5 = std::make_shared<Contour>(50, contour5Points);

    std::map<int, std::shared_ptr<Contour>> contourList;
    contourList[40] = contour4;
    contourList[10] = contour1;
    contourList[50] = contour5;
    contourList[30] = contour3;
    contourList[20] = contour2;

    solver.setContourList(contourList);
    solver.solve();
    auto result = solver.getTravelOrder();

    std::cout << "Alternate travel order size = " << result.size() << std::endl;

    
    ASSERT_EQ(result.size(), 5);
    std::vector<std::pair<int, bool>> expected_result = {
      {20, false},
      {40, true},
      {50, true},
      {30, true},
      {10, true},
  };
  
  EXPECT_EQ(result, expected_result);
  

  }
  catch (const std::exception &e) {
    std::cerr << "Exception: " << e.what() << std::endl;
  }

  rclcpp::shutdown();
}
