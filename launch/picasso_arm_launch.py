#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
  ld = LaunchDescription()

  # Launch arguments
  declare_ur_mock_hardware = DeclareLaunchArgument(
    'ur_mock_hardware',
    default_value='true',
    description='Whether to use mock hardware'
  )

  declare_launchRVIZ = DeclareLaunchArgument(
    'launch_rviz',
    default_value='true',
    description='Whether launch RVIZ'
  )
  
  # Launch configurations
  urType = 'ur3e'
  urIP = '111.111.111.111'
  urController = 'joint_trajectory_controller'
  urMockHardware = LaunchConfiguration('ur_mock_hardware')
  urStartDelay = 3.0  # [sec].
  moveItStartDelay = 3.0  # [sec].
  launchRVIZ = LaunchConfiguration('launch_rviz')

  # nodes
  picasso_arm = Node(
    package="picasso_bot",
    executable="picasso_arm",
    name="picasso_arm",
    output="screen"
  )

  # Allow rviz if not using mock hardware.
  if (launchRVIZ == 'true') and (urMockHardware == 'false'):
    ur_sim_rviz = 'true'

  else:
    ur_sim_rviz = 'false'

  ur_sim_gz = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      os.path.join(get_package_share_directory("ur_simulation_gazebo"), "launch"),
      "/ur_sim_control.launch.py"
    ]),

    launch_arguments={
      "ur_type": urType,
      "initial_joint_controller": urController,
      "launch_rviz": ur_sim_rviz, 
    }.items(),

    condition=IfCondition(urMockHardware)
    )
    

  ur_driver = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      os.path.join(get_package_share_directory('ur_robot_driver'), 'launch'),
      '/ur_control.launch.py'
    ]),

    launch_arguments={
      'ur_type': urType,
      'robot_ip': urIP,
      'launch_rviz': 'false',
      'initial_joint_controller': urController,
      'use_fake_hardware': urMockHardware
    }.items()
  )

  move_it = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      os.path.join(get_package_share_directory("ur_moveit_config"), "/launch"), 
      "/ur_moveit.launch.py"
    ]),
    
    launch_arguments={
        "ur_type": urType,
        "use_sim_time": urMockHardware,
        "launch_rviz": launchRVIZ,
        "use_fake_hardware": urMockHardware,
    }.items()
  )

  # Start up sequence - add delay for URDriver and MoveIt to start.
  delayed_start_move_it = TimerAction(
    period=urStartDelay,
    actions=[move_it]
  )

  delayed_start_picasso_arm = TimerAction(
    period=moveItStartDelay + urStartDelay,  # Add both delays to ensure proper sequence
    actions=[picasso_arm]
  )

  # Add the declare argument action
  ld.add_action(declare_ur_mock_hardware)
  ld.add_action(declare_launchRVIZ)
  
  # Run remaining nodes.
  ld.add_action(ur_sim_gz)
  ld.add_action(ur_driver)
  ld.add_action(delayed_start_move_it)
  ld.add_action(delayed_start_picasso_arm)

  return ld