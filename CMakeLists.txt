cmake_minimum_required(VERSION 3.8)
project(picasso_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Required for QT
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(realsense2 REQUIRED)
find_package(realsense2_camera_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Multimedia MultimediaWidgets)
find_package(ros_image_to_qimage REQUIRED)
find_package(rcl_interfaces REQUIRED)

# Add include directories
# qt5: "/usr/include/x86_64-linux-gnu/qt5/**"
# ROS: "/opt/ros/humble/include/**"
# openCV: "/usr/include/opencv4/**"

include_directories(
  src/picasso_arm
  src/picasso_eyes
  src/picasso_ui
)

# Arm
set(picasso_arm_dep
  ament_index_cpp
  rclcpp
  sensor_msgs
  geometry_msgs
  OpenCV
  cv_bridge
  image_transport
  realsense2
  realsense2_camera_msgs
  moveit_ros_planning_interface
)

set(picasso_arm_src
  src/picasso_arm/arm_main.cpp
  src/picasso_arm/picasso_arm.cpp
  src/picasso_eyes/Contour.cpp
)

add_executable(picasso_arm ${picasso_arm_src})
ament_target_dependencies(picasso_arm ${picasso_arm_dep})

# Eyes
set(picasso_eyes_dep 
  ament_index_cpp
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  OpenCV
  cv_bridge
  image_transport
  realsense2
  realsense2_camera_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

set(picasso_eyes_src
  src/picasso_eyes/eyes_main.cpp
  src/picasso_eyes/picasso_eyes.cpp
  src/picasso_eyes/Contour.cpp
  src/picasso_eyes/image_controller.cpp
  src/picasso_eyes/salesman_solver.cpp
)

add_executable(picasso_eyes ${picasso_eyes_src})
ament_target_dependencies(picasso_eyes ${picasso_eyes_dep} geometry_msgs)

# UI
set(picasso_ui_dep
  ament_index_cpp 
  rclcpp
  std_msgs
  Qt5
  ros_image_to_qimage
  OpenCV
  cv_bridge
  image_transport
  sensor_msgs
  visualization_msgs
  realsense2
  realsense2_camera_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

set (picasso_ui_src
  src/picasso_ui/mainwindow.cpp
  src/picasso_ui/ui_main.cpp
)

set(UI_FILES
  src/picasso_ui/mainwindow.ui
)

add_executable(picasso_ui ${picasso_ui_src} ${UI_FILES})
ament_target_dependencies(picasso_ui ${picasso_ui_dep})
target_link_libraries(picasso_ui
  Qt5::Widgets
  Qt5::Multimedia
  Qt5::MultimediaWidgets
)

# UTesting
find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_salesman_solver
  tests/test_salesman_solver.cpp
  src/picasso_eyes/salesman_solver.cpp
  src/picasso_eyes/Contour.cpp
)

target_link_libraries(test_salesman_solver
  gtest_main
)

ament_target_dependencies(test_salesman_solver
  rclcpp
  std_msgs
  ros_image_to_qimage
  OpenCV
  cv_bridge
  image_transport
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  realsense2
  realsense2_camera_msgs
)



install(TARGETS
  picasso_arm
  picasso_eyes
  picasso_ui
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  config
  local # Comment out if causing build issues
  launch
  rviz
  worlds
  tests
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()