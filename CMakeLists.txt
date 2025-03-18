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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
#find_package(librealsense2 REQUIRED)

# Add include directories
include_directories(
  src/picasso_arm
  src/picasso_eyes
  src/picasso_ui
)

# Arm
set(picasso_arm_dep "ament_index_cpp" "rclcpp")
add_library(picasso_arm src/picasso_arm/picasso_arm.cpp)
ament_target_dependencies(picasso_arm ${picasso_arm_dep})

# Eyes
set(picasso_eyes_dep "ament_index_cpp" "rclcpp" "sensor_msgs" "OpenCV")
add_library(picasso_eyes src/picasso_eyes/picasso_eyes.cpp)
ament_target_dependencies(picasso_eyes ${picasso_eyes_dep})

# UI
set(picasso_ui_dep "ament_index_cpp" "rclcpp")
add_library(picasso_ui src/picasso_ui/picasso_ui.cpp)
ament_target_dependencies(picasso_ui ${picasso_ui_dep})

# Make executable
set(picasso_core_dep "ament_index_cpp" "rclcpp")
add_executable(picasso_core src/main.cpp)
ament_target_dependencies(picasso_core ${picasso_core_dep})
target_link_libraries(picasso_core picasso_arm picasso_eyes picasso_ui)

install(TARGETS
  picasso_arm
  picasso_eyes
  picasso_ui
  picasso_core
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
  config
  launch
  rviz
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
