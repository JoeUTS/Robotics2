#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

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

  # Sim only
  ur_sim_driver = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      os.path.join(get_package_share_directory("ur_simulation_gazebo"), "launch"),
      "/ur_sim_control.launch.py"
    ]),

    launch_arguments={
      "ur_type": urType,
      "gazebo_gui": urMockHardware,
      "launch_rviz": "false"
    }.items(),

    condition=IfCondition(urMockHardware)
    )

  # Non-Sim only
  ur_driver = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      os.path.join(get_package_share_directory('ur_robot_driver'), 'launch'),
      '/ur_control.launch.py'
    ]),

    launch_arguments={
      'ur_type': urType,
      'robot_ip': urIP,
      'launch_rviz': "false",
      'use_fake_hardware': urMockHardware,
      'controller_spawner_timeout': '60'
    }.items(),

    condition=UnlessCondition(urMockHardware)
  )

  move_it = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      os.path.join(get_package_share_directory("picasso_bot"), "launch"), 
      "/picasso_arm_moveit_config.launch.py"
    ]),
    
    launch_arguments={
        "ur_type": urType,
        "ur_mock_hardware": urMockHardware,
        "robot_ip": urIP,
        "use_sim_time": urMockHardware,
        "launch_rviz": launchRVIZ,
        "use_fake_hardware": urMockHardware,
    }.items()
  )

  # Node start delays.
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
  
  # Select driver.
  if urMockHardware == 'true':
    ld.add_action(ur_sim_driver)
  else:
    ld.add_action(ur_driver)

  ld.add_action(delayed_start_move_it)
  ld.add_action(delayed_start_picasso_arm)

  return ld