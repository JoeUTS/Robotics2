#!/usr/bin/env python3

from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration


def generate_launch_description():
    declare_ur_mock_hardware = DeclareLaunchArgument(
      'ur_mock_hardware',
      default_value='true',
      description='Whether to use mock hardware'
    )
    ur_Mock_Hardware = LaunchConfiguration('ur_mock_hardware')
    
    declare_rviz = DeclareLaunchArgument(
      'launch_rviz',
      default_value="true",
      description='Enable RVIZ for arm module'
    )
    launch_rviz = LaunchConfiguration('launch_rviz')

    declare_ui_delay = DeclareLaunchArgument(
      'ui_start_delay',
      default_value='5.0',
      description='Delay before launching UI [sec]'
    )
    ui_start_delay = LaunchConfiguration('ui_start_delay')

    declare_ur_type = DeclareLaunchArgument(
      'ur_type',
      default_value='ur3e',
      description='UR type (ur3e, ur5e, ur10e, ect)'
    )
    ur_type = LaunchConfiguration('ur_type')

    declare_ur_IP = DeclareLaunchArgument(
      'ur_IP',
      default_value='111.111.111.111',
      description='IP address of the UR robot'
    )
    ur_IP = LaunchConfiguration('ur_IP')


    # Sub-modules
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
        'launch_rviz': launch_rviz
      }.items()
    )

    picasso_arm = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        PathJoinSubstitution([
          FindPackageShare('picasso_bot'), 
          'launch',
          'picasso_arm_launch.py'
        ])
      ]),
      launch_arguments={
        'ur_type': ur_type,
        'ur_IP': ur_IP,
        'launch_rviz': launch_rviz,
        'ur_mock_hardware': ur_Mock_Hardware
      }.items()
    )


    delayed_start_ui = TimerAction(
      period=ui_start_delay,
      actions=[picasso_ui]
    )

    ld = LaunchDescription()
    ld.add_action(declare_rviz)
    ld.add_action(declare_ur_mock_hardware)
    ld.add_action(declare_ui_delay)
    ld.add_action(declare_ur_type)
    ld.add_action(declare_ur_IP)

    ld.add_action(delayed_start_ui)
    ld.add_action(picasso_eyes)
    ld.add_action(picasso_arm)

    return ld

