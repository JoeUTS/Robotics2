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
  realSenseStartDelay = 5.0  # [sec].
 
  # launch parameters
  rviz_config_file = os.path.join(
    get_package_share_directory('picasso_bot'),
    'rviz',
    'config_test_picasso_eyes.rviz'
  )
 
  # nodes
  picasso_eyes = Node(
    package="picasso_bot",
    executable="picasso_eyes",
    name="node_picasso_eyes",
    output="screen",
    emulate_tty=True
  )

  realsense_camera = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      os.path.join(get_package_share_directory('realsense2_camera'), 'launch'),
      '/rs_launch.py'
    ]),
    launch_arguments={
      'enable_rgbd': 'true',
      'rgb_camera.color_profile': '640x480x30',
      'depth_module.depth_profile': '640x480x30',
      'enable_sync': 'true',
      'align_depth.enable': 'true',
      'enable_color': 'true',
      'enable_depth': 'true',
      "rgb_camera.enable_auto_exposure": 'true',
      "depth_module.enable_auto_exposure": 'true',
      "initial_reset": 'true'
    }.items()
  )

  rviz = Node(
    package="rviz2",
    executable="rviz2",
    name="rviz2",
    output="log",
    arguments=["-d", rviz_config_file]
  )

  delayed_start_picasso_eyes = TimerAction(
    period=realSenseStartDelay,
    actions=[picasso_eyes]
  )

  delayed_start_rviz = TimerAction(
    period=realSenseStartDelay,
    actions=[rviz]
  )

  # add actions to launch
  ld.add_action(realsense_camera)
  ld.add_action(delayed_start_picasso_eyes)
  ld.add_action(delayed_start_rviz)

  return ld

