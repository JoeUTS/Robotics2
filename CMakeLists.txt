cmake_minimum_required(VERSION 3.10)
project(picasso_bot)

if (CMAKE_VERSION VERSION_GREATER_EQUAL "3.11")
  # CMP0071: Let AUTOMOC and AUTOUIC process GENERATED files.
  cmake_policy(SET CMP0071 NEW)
endif()
if (CMAKE_VERSION VERSION_GREATER_EQUAL "3.13")
  # CMP0167: The FindBoost module is removed.
  cmake_policy(SET CMP0167 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Required for QT
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED) # Custom service
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
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
find_package(rosidl_default_generators REQUIRED) # Custom service
find_package(builtin_interfaces REQUIRED) # Custom service

# Add include directories
# qt5: "/usr/include/x86_64-linux-gnu/qt5/**"
# ROS: "/opt/ros/humble/include/**"
# openCV: "/usr/include/opencv4/**"
# ROS2_ws package: "/home/<username>/ros2_ws/install/**"

# Custom services
include_directories(
  include
  ${CMAKE_SOURCE_DIR}/include
)
rosidl_generate_interfaces(${PROJECT_NAME}
  srv/GetPoseArray.srv
  srv/GetImage.srv
  DEPENDENCIES builtin_interfaces geometry_msgs sensor_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target 
  ${PROJECT_NAME} 
  rosidl_typesupport_cpp
)

# Arm
set(picasso_arm_src
  src/picasso_arm/arm_main.cpp
  src/picasso_arm/picasso_arm.cpp
)

add_executable(picasso_arm ${picasso_arm_src})
ament_target_dependencies(picasso_arm
  ament_index_cpp
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  moveit_ros_planning_interface
)
target_link_libraries(picasso_arm "${cpp_typesupport_target}")

# Eyes
set(picasso_eyes_src
  src/picasso_eyes/eyes_main.cpp
  src/picasso_eyes/picasso_eyes.cpp
  src/picasso_eyes/image_controller.cpp
  src/picasso_eyes/VisulisationSettings.cpp
  src/common/Contour.cpp
  src/salesman_solver/salesman_solver.cpp
)

add_executable(picasso_eyes ${picasso_eyes_src})
ament_target_dependencies(picasso_eyes
  ament_index_cpp
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  std_srvs
  OpenCV
  cv_bridge
  image_transport
  realsense2
  realsense2_camera_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(picasso_eyes "${cpp_typesupport_target}")

# UI
set (picasso_ui_src
  src/picasso_ui/mainwindow.cpp
  src/picasso_ui/ui_main.cpp
)
set(UI_FILES
  src/picasso_ui/mainwindow.ui
)
add_executable(picasso_ui ${picasso_ui_src} ${UI_FILES})
ament_target_dependencies(picasso_ui 
  ament_index_cpp 
  rclcpp
  std_msgs
  std_srvs
  Qt5
  ros_image_to_qimage
  OpenCV
  cv_bridge
  image_transport
  sensor_msgs
)
target_link_libraries(picasso_ui
  Qt5::Widgets
  Qt5::Multimedia
  Qt5::MultimediaWidgets
  "${cpp_typesupport_target}"
)

# UTesting
find_package(ament_cmake_gtest REQUIRED)
ament_add_gtest(test_salesman_solver
  tests/test_salesman_solver.cpp
  src/salesman_solver/salesman_solver.cpp
  src/common/Contour.cpp
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
target_link_libraries(test_salesman_solver
  gtest_main
  "${cpp_typesupport_target}"
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
  #local # Comment out if causing build issues
  launch
  rviz
  worlds
  srv
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()