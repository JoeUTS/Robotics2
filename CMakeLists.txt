cmake_minimum_required(VERSION 3.8)
project(PicassoBot)

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

# Add submodules
# Arm
set(PicassoArm_dep "ament_index_cpp" "rclcpp")

set(PicassoArm_SRCS
  src/PicassoArm.cpp
)

set(PicassoArm_HDRS
  include/PicassoArm.h
)

add_library(PicassoArm ${PicassoArm_SRCS} ${PicassoArm_HDRS}) # If any issues, remove the headers argument.
ament_target_dependencies(PicassoArm ${PicassoArm_dep})

# Eyes
set(PicassoEyes_dep "ament_index_cpp" "rclcpp")

set(PicassoEyes_SRCS
  src/PicassoEyes.cpp
)

set(PicassoEyes_HDRS
  include/PicassoEyes.h
)

add_library(PicassoEyes ${PicassoEyes_SRCS} ${PicassoEyes_HDRS}) # If any issues, remove the headers argument.
ament_target_dependencies(PicassoEyes ${PicassoEyes_dep})

# UI
set(PicassoUI_dep "ament_index_cpp" "rclcpp")

set(PicassoUI_SRCS
  src/PicassoUI.cpp
)

set(PicassoUI_HDRS
  include/PicassoUI.h
)

add_library(PicassoUI ${PicassoUI_SRCS} ${PicassoUI_HDRS}) # If any issues, remove the headers argument.
ament_target_dependencies(PicassoUI ${PicassoUI_dep})


# Make executables
set(PicassoBot_dep "ament_index_cpp" "rclcpp")

set(PicassoBot_SRCS
  src/main.cpp
)

set(PicassoBot_HDRS
  
)

add_executable(PicassoBot ${PicassoBot_SRCS} ${PicassoBot_HDRS}) # If any issues, remove the headers argument.

install(TARGETS
  PicassoArm
  PicassoEyes
  PicassoUI
  PicassoBot
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

ament_package()