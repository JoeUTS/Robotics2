# Robotics2 class GIT

Picassobot project for Robotics 2

Requirements:
Ubuntu 22.04 LTS
ROS2 Humble
https://docs.ros.org/en/humble/index.html

RealSence SDK2.
https://dev.intelrealsense.com/docs/compiling-librealsense-for-linux-ubuntu-guide

ROS Wrapper for Intel RealSense Cameras.
https://github.com/IntelRealSense/realsense-ros

OpenCV.
https://opencv.org/

MoveIT2.
https://moveit.picknik.ai/main/index.html

URDriver.
https://www.universal-robots.com/developer/communication-protocol/ros-and-ros2-driver/

ROS2 Control
https://control.ros.org/rolling/doc/getting_started/getting_started.html

GZ sim (fortress)
https://gazebosim.org/docs/fortress/install_ubuntu/

ROS2 GZ control
https://github.com/ros-controls/gz_ros2_control
If you get errors while launching, try updating the ur_sim_control.launch.py section with:

"z_launch_description = IncludeLaunchDescription(
  PythonLaunchDescriptionSource(
    [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
  ),
  launch_arguments={
    "gz_args": [" -r -v 4 ", world_file] if gazebo_gui.perform(context) == 'true' else [" -s -r -v 4 ", world_file]
  }.items(),
)"

You may also need to replace "force_abs_paths="true"" with "sim_gazebo="true"" in ur_simulation_gz/urdf/ur_gz.urdf.xacro

ur_simulation_gz
https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_simulation_gz/ur_simulation_gz/doc/installation.html

QT (Version?)
https://www.qt.io/product/development-tools

