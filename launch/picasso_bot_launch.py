#!/usr/bin/env python3

from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()

    declare_ur_mock_hardware = DeclareLaunchArgument(
      'ur_mock_hardware',
      default_value='true',
      description='Whether to use mock hardware'
    )

    declare_rviz_eyes = DeclareLaunchArgument(
      'launch_rviz_eyes',
      default_value="False",
      description='Enable RVIZ for eyes module'
    )
    
    declare_rviz_arm = DeclareLaunchArgument(
      'launch_rviz_arm',
      default_value="False",
      description='Enable RVIZ for arm module'
    )

    ur_Mock_Hardware = LaunchConfiguration('ur_mock_hardware')
    rviz_eyes = LaunchConfiguration('launch_rviz_eyes')
    rviz_arm = LaunchConfiguration('launch_rviz_arm')

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
        'launch_rviz': rviz_eyes
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
        'launch_rviz': rviz_arm,
        'ur_mock_hardware': ur_Mock_Hardware
      }.items()
    )

    ld.add_action(declare_rviz_eyes)
    ld.add_action(declare_rviz_arm)
    ld.add_action(declare_ur_mock_hardware)
    ld.add_action(picasso_ui)
    ld.add_action(picasso_eyes)
    #ld.add_action(picasso_arm)

    return ld

