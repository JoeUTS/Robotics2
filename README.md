# Robotics2 class GIT

# Picassobot project for Robotics 2

## Requirements:
Ubuntu 22.04 LTS
### ROS2 Humble
> [https://docs.ros.org/en/humble/index.html](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) <br/> 

### Gazebo Classic
> sudo apt-get install gazebo* <br/> 

### RealSence SDK2.
> sudo mkdir -p /etc/apt/keyrings <br/> 
> curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null <br/> 
> echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \ <br/> 
> sudo tee /etc/apt/sources.list.d/librealsense.list <br/> 
> sudo apt-get update <br/> 
> sudo apt-get install librealsense2-dkms <br/> 
> sudo apt-get install librealsense2-utils <br/> 
> sudo apt-get install librealsense2-dev <br/> 
> sudo apt-get install librealsense2-dbg <br/> 

### ROS Wrapper for Intel RealSense Cameras.
> sudo apt install ros-humble-realsense2-* <br/> 

### OpenCV.
> sudo apt install libopencv-dev <br/>
> sudo apt-get install libqt5multimedia5 libqt5multimedia5-plugins

### MoveIT2.
> sudo apt install ros-humble-moveit <br/> 

### URDriver.
> sudo apt-get install ros-humble-ur <br/> 

### Gazebo Simulator(UR Simulation Compatible).
> https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/tree/humble <br/> 

### ur_simulation_gz
!! file will need to be in ros2_ws/src either directly or as symbolic link. <br/> 
> git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation.git <br/> 
> rosdep update && rosdep install --ignore-src --from-paths . -y <br/> 
> cd ros2_ws/src <br/> 
> colcon build --symlink-install <br/> 

### QT
> sudo apt-get install build-essential libgl1-mesa-dev <br/>
> sudo apt-get install qtmultimedia5-dev <br/>

### ros_image_to_qimage
> sudo apt install ros-humble-ros-image-to-qimage <br/> 
