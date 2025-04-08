#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
  ld = LaunchDescription()

  # settings
  urType = 'ur3e'
  urIP = '192.168.56.101:6080'
  urController = 'joint_trajectory_controller'
  urMockHardware = 'true'
  urStartDelay = 1.5  # [sec].
  moveItStartDelay = 1.5  # [sec].
  launchRVIZ = 'true'

  # nodes
  picasso_arm = Node(
    package="picasso_bot",
    executable="picasso_arm",
    name="picasso_arm",
    output="screen"
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
      os.path.join(get_package_share_directory('ur_moveit_config'), 'launch'),
      '/ur_moveit.launch.py'
    ]),
    launch_arguments={
      'ur_type': urType,
      'launch_rviz': launchRVIZ,
      'use_fake_hardware': urMockHardware
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

  ld.add_action(ur_driver)
  ld.add_action(delayed_start_move_it)
  ld.add_action(delayed_start_picasso_arm)

  return ld