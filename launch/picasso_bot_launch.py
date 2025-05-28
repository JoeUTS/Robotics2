#!/usr/bin/env python3

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition


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

    declare_rviz_start_delay = DeclareLaunchArgument(
      'rviz_Start_Delay',
      default_value='6.0',
      description='Delay before launching RVIZ'
    )
    rviz_Start_Delay = LaunchConfiguration('ur_Start_Delay')

    declare_rviz_config_file = DeclareLaunchArgument(
      "rviz_config_file",
      default_value=PathJoinSubstitution([
        FindPackageShare('picasso_bot'),
        "rviz", 
        "config_test_picasso_arm.rviz"
      ]),
    )
    rviz_config_file = LaunchConfiguration('rviz_config_file')

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
        'launch_rviz': 'false'
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
        'launch_rviz': 'false',
        'ur_mock_hardware': ur_Mock_Hardware
      }.items()
    )

    rviz_node = Node(
      package="rviz2",
      condition=IfCondition(launch_rviz),
      executable="rviz2",
      name="rviz2",
      output="screen",
      arguments=["-d", rviz_config_file],
    )


    delayed_start_ui = TimerAction(
      period=ui_start_delay,
      actions=[picasso_ui],
      cancel_on_shutdown=True
    )

    delayed_start_rviz = TimerAction(
      period=rviz_Start_Delay,
      actions=[rviz_node],
      cancel_on_shutdown=True
    )

    ld = LaunchDescription()
    ld.add_action(declare_rviz)
    ld.add_action(declare_rviz_start_delay)
    ld.add_action(declare_rviz_config_file)
    ld.add_action(declare_ur_mock_hardware)
    ld.add_action(declare_ui_delay)
    ld.add_action(declare_ur_type)
    ld.add_action(declare_ur_IP)

    ld.add_action(delayed_start_ui)
    ld.add_action(delayed_start_rviz)
    ld.add_action(picasso_eyes)
    ld.add_action(picasso_arm)

    return ld
