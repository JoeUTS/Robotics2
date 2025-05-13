#!/usr/bin/env python3

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition, UnlessCondition

# manual launch - Cant seem to get gazebo launching from combined launch - Joseph.
# ros2 launch picasso_bot picasso_arm_ur_sim_control.launch.py ur_type:=ur3e gazebo_gui:=true launch_rviz:=false use_sim_time:=true
# ros2 launch picasso_bot picasso_arm_moveit_config.launch.py ur_type:=ur3e ur_mock_hardware:=true robot_ip:=111.111.111.111 use_sim_time:=true launch_rviz:=true

def generate_launch_description():
    ld = LaunchDescription()

    # Launch arguments
    declare_ur_mock_hardware = DeclareLaunchArgument(
      'ur_mock_hardware',
      default_value='true',
      description='Whether to use mock hardware'
    )

    declare_launch_RVIZ = DeclareLaunchArgument(
      'launch_rviz',
      default_value='true',
      description='Whether launch RVIZ'
    )
    
    # Launch configurations
    launch_RVIZ = LaunchConfiguration('launch_rviz')
    ur_Mock_Hardware = LaunchConfiguration('ur_mock_hardware')
    ur_Type = 'ur3e'
    ur_IP = '111.111.111.111'
    ur_Start_Delay = 1.0       # [sec].
    move_It_Start_Delay = 1.0   # [sec].

    # nodes
    picasso_arm = Node(
      package="picasso_bot",
      executable="picasso_arm",
      name="picasso_arm",
      output="screen",
      parameters=[
          {'use_sim_time': ur_Mock_Hardware}
      ]
    )

    # Sim only
    ur_sim_driver = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        PathJoinSubstitution([
          FindPackageShare("picasso_bot"),
          "launch",
          "picasso_arm_moveit_config.launch.py"
          ])
      ]),

      launch_arguments={
        "ur_type": ur_Type,
        "gazebo_gui": "true",
        "launch_rviz": "false", # move it will launch rviz
        "use_sim_time": "true"
      }.items(),
      condition=IfCondition(ur_Mock_Hardware)
    )

    # Non-Sim only
    ur_driver = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        PathJoinSubstitution([
          FindPackageShare('picasso_bot'),
          'launch',
          'picasso_arm_ur_control.launch.py'
          ])
      ]),
      launch_arguments={
        'ur_type': ur_Type,
        'robot_ip': ur_IP,
        'launch_rviz': "false", # move it will launch rviz
        'use_fake_hardware': "false",
        'controller_spawner_timeout': "60",
        'initial_joint_controller': 'joint_trajectory_controller'
      }.items(),
      condition=UnlessCondition(ur_Mock_Hardware)
    )

    move_it = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        PathJoinSubstitution([
          FindPackageShare("picasso_bot"), 
          "launch", 
          "picasso_arm_moveit_config.launch.py"
          ])
      ]),
      launch_arguments={
          "ur_type": ur_Type,
          "ur_mock_hardware": ur_Mock_Hardware,
          "robot_ip": ur_IP,
          "use_sim_time": ur_Mock_Hardware,
          "launch_rviz": launch_RVIZ
      }.items()
    )

    # Node start delays.
    delayed_start_move_it = TimerAction(
      period=ur_Start_Delay,
      actions=[move_it]
    )

    delayed_start_picasso_arm = TimerAction(
      period=move_It_Start_Delay + ur_Start_Delay,  # Add both delays to ensure proper sequence
      actions=[picasso_arm]
    )

    # Add the declare argument action
    ld.add_action(declare_ur_mock_hardware)
    ld.add_action(declare_launch_RVIZ)
    ld.add_action(ur_sim_driver)
    ld.add_action(ur_driver)
    ld.add_action(delayed_start_move_it)
    ld.add_action(delayed_start_picasso_arm)

    return ld