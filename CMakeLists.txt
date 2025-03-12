cmake_minimum_required(VERSION 3.8)
project(picasso_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++20 - Dunno if this will work. Revert to c++17 if it doesnt.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

#find_package(librealsense2  REQUIRED)


# Add submodules
# Arm
set(picasso_arm_dep "ament_index_cpp" "rclcpp")

set(picasso_arm_SRCS
  src/picasso_arm/picasso_arm.cpp
)

set(picasso_arm_HDRS
  include/picasso_arm.h
)

add_library(picasso_arm ${picasso_arm_SRCS}) # If any issues, remove the headers argument.
ament_target_dependencies(picasso_arm ${picasso_arm_dep})

# Eyes
set(picasso_eyes_dep "ament_index_cpp" "rclcpp" "sensor_msgs" "OpenCV")

set(picasso_eyes_SRCS
  src/picasso_eyes/picasso_eyes.cpp
)

set(picasso_eyes_HDRS
  include/picasso_eyes.h
)

add_library(picasso_eyes ${picasso_eyes_SRCS}) # If any issues, remove the headers argument.
ament_target_dependencies(picasso_eyes ${picasso_eyes_dep})

# UI
set(picasso_ui_dep "ament_index_cpp" "rclcpp")

set(picasso_ui_SRCS
  src/picasso_ui/picasso_ui.cpp
)

set(picasso_ui_HDRS
  include/picasso_ui.h
)

add_library(picasso_ui ${picasso_ui_SRCS}) # If any issues, remove the headers argument.
ament_target_dependencies(picasso_ui ${picasso_ui_dep})


# Make executable
set(picasso_core_dep "ament_index_cpp" "rclcpp")

set(picasso_core_SRCS
  src/main.cpp
)

set(picasso_core_HDRS
  
)

add_executable(picasso_core ${picasso_core_SRCS}) # If any issues, remove the headers argument.
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
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()