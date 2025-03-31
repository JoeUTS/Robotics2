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
# Core
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


# Eyes
find_package(ament_index_cpp)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(realsense2 REQUIRED)
find_package(realsense2_camera_msgs REQUIRED)

# Arm

# UI

# Add include directories
include_directories(
  src/picasso_arm
  src/picasso_eyes
  src/picasso_ui
)

# Arm
set(picasso_arm_dep
  ament_index_cpp
  rclcpp
  )

set(picasso_arm_src
  src/picasso_arm/arm_main.cpp
  src/picasso_arm/picasso_arm.cpp
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
  src/picasso_eyes/image_controller.cpp
)

add_executable(picasso_eyes ${picasso_eyes_src})
ament_target_dependencies(picasso_eyes ${picasso_eyes_dep})
#target_link_libraries(picasso_eyes ) # probably not needed. Holding onto incase needing to make imageControler a static library.

# UI
set(picasso_ui_dep 
  ament_index_cpp 
  rclcpp
  )

set (picasso_ui_src
  src/picasso_ui/ui_main.cpp
  src/picasso_ui/picasso_ui.cpp
)

add_executable(picasso_ui ${picasso_ui_src})
ament_target_dependencies(picasso_ui ${picasso_ui_dep})


install(TARGETS
  picasso_arm
  picasso_eyes
  picasso_ui
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
  config
  launch
  rviz
  worlds
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
