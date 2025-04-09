# Robotics2 class GIT

Picassobot project for Robotics 2

Requirements:
Ubuntu 22.04 LTS
ROS2 Humble
[https://docs.ros.org/en/humble/index.html](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

Gazebo Classic
sudo apt-get install gazebo*

RealSence SDK2.
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg

ROS Wrapper for Intel RealSense Cameras.
sudo apt install ros-humble-realsense2-*

OpenCV.
sudo apt install libopencv-dev

MoveIT2.
sudo apt install ros-humble-moveit

URDriver.
sudo apt-get install ros-humble-ur

ur_simulation_gz
!! file will need to be in ros2_ws/src either directly or as symbolic link.
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation.git
rosdep update && rosdep install --ignore-src --from-paths . -y
cd ros2_ws/src
colcon build --symlink-install

QT
sudo apt-get install build-essential libgl1-mesa-dev

