#!/usr/bin/env python3

from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
  ld = LaunchDescription()

  picasso_ui = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      PathJoinSubstitution([FindPackageShare('picasso_bot'), 
        'launch',
        'picasso_ui_launch.py'
      ])
    ]),
    launch_arguments={
      
    }.items()
  )

  picasso_eyes = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      PathJoinSubstitution([
        FindPackageShare('picasso_bot'), 
        'launch',
        'picasso_eyes_launch.py'
      ])
    ]),
    launch_arguments={
      'use_rviz': 'false'
    }.items()
  )

  picasso_arm = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      PathJoinSubstitution([
        FindPackageShare('picasso_bot'), 
        'launch',
        '/picasso_arm_launch.py'
      ])
    ]),
    launch_arguments={
      
    }.items()
  )

  ld.add_action(picasso_ui)
  ld.add_action(picasso_eyes)
  #ld.add_action(picasso_arm)

  return ld

