#!/usr/bin/env python3

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression

def generate_launch_description():
    ld = LaunchDescription()

    # settings
    realSenseStartDelay = 7.0  # [sec].
  
    # launch parameters
    rviz_config_file = PathJoinSubstitution([
      FindPackageShare('picasso_bot'),
      'rviz',
      'config_test_picasso_eyes.rviz'
    ])

    
    declare_use_rviz = DeclareLaunchArgument(
      'use_rviz',
      default_value="true",
      description='Whether to open rviz'
    )

    use_rviz = LaunchConfiguration('use_rviz')
  
    # nodes
    picasso_eyes = Node(
      package="picasso_bot",
      executable="picasso_eyes",
      name="node_picasso_eyes",
      output="screen",
      emulate_tty=True
    )

    rviz_node = Node(
      package="rviz2",
      condition=IfCondition(use_rviz),
      executable="rviz2",
      name="rviz2",
      output="screen",
      arguments=["-d", rviz_config_file],
    )

    realsense_camera = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            FindPackageShare('realsense2_camera'), 
            'launch',
            'rs_launch.py'
        ])
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

    delayed_start_picasso_eyes = TimerAction(
      period=realSenseStartDelay,
      actions=[picasso_eyes, rviz_node]
    )

    # add actions to launch
    ld.add_action(declare_use_rviz)
    ld.add_action(realsense_camera)
    ld.add_action(delayed_start_picasso_eyes)


    return ld

