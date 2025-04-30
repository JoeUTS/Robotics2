#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
  ld = LaunchDescription()

  picasso_ui = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      os.path.join(get_package_share_directory('picasso_bot'), 'launch'),
      '/picasso_ui_launch.py'
    ]),
    launch_arguments={
      
    }.items()
  )

  picasso_eyes = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      os.path.join(get_package_share_directory('picasso_bot'), 'launch'),
      '/picasso_eyes_launch.py'
    ]),
    launch_arguments={
      
    }.items()
  )

  picasso_arm = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      os.path.join(get_package_share_directory('picasso_bot'), 'launch'),
      '/picasso_arm_launch.py'
    ]),
    launch_arguments={
      
    }.items()
  )

  ld.add_action(picasso_ui)
  ld.add_action(picasso_eyes)
  #ld.add_action(picasso_arm)

  return ld

