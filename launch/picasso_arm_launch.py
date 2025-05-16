#!/usr/bin/env python3

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition

# manual launch - Cant seem to get gazebo launching from combined launch - Joseph.
# ros2 launch picasso_bot picasso_arm_ur_sim_control.launch.py ur_type:=ur3e gazebo_gui:=true launch_rviz:=false use_sim_time:=true
# ros2 launch picasso_bot picasso_arm_moveit_config.launch.py ur_type:=ur3e ur_mock_hardware:=true robot_ip:=111.111.111.111 use_sim_time:=true launch_rviz:=true

def generate_launch_description():
    # Launch arguments
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

    declare_ur_start_delay = DeclareLaunchArgument(
      'ur_Start_Delay',
      default_value='3.0',
      description='Delay before launching UR drivers'
    )
    ur_Start_Delay = LaunchConfiguration('ur_Start_Delay')

    declare_ur_mock_hardware = DeclareLaunchArgument(
      'mock_hardware',
      default_value='true',
      description='Whether to use mock hardware'
    )
    ur_Mock_Hardware = LaunchConfiguration('mock_hardware')

    declare_launch_rviz = DeclareLaunchArgument(
      'launch_rviz',
      default_value='true',
      description='Whether launch RVIZ'
    )
    launch_rviz = LaunchConfiguration('launch_rviz')

    declare_rviz_config_file = DeclareLaunchArgument(
      "rviz_config_file",
      default_value=PathJoinSubstitution([
        FindPackageShare('picasso_bot'),
        "rviz", 
        "config_test_picasso_arm.rviz"
      ]),
    )
    rviz_config_file = LaunchConfiguration('rviz_config_file')


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

    rviz_node = Node(
      package="rviz2",
      condition=IfCondition(launch_rviz),
      executable="rviz2",
      name="rviz2",
      output="screen",
      arguments=["-d", rviz_config_file],
    )


    # Sim driver
    ur_sim_driver = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        FindPackageShare("ur_simulation_gazebo"),
        "/launch",
        "/ur_sim_control.launch.py"
      ]),
      launch_arguments={
        "ur_type": ur_type,
        "launch_rviz": 'false'
      }.items(),
      condition=IfCondition(ur_Mock_Hardware)
    )

    # Non-Sim only
    ur_driver = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        PathJoinSubstitution([
          FindPackageShare('ur_robot_driver'),
          'launch',
          'ur_control.launch.py'
          ])
      ]),
      launch_arguments={
        "ur_type":ur_type,
        "robot_ip": ur_IP,
        "controller_spawner_timeout": "30",
        "launch_rviz": 'false'
      }.items(),
      condition=UnlessCondition(ur_Mock_Hardware)
    )

    moveit = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
          FindPackageShare("ur_moveit_config"),
          "/launch",
          "/ur_moveit.launch.py"]
      ),
      launch_arguments={
          "ur_type": ur_type,
          "use_sim_time": ur_Mock_Hardware,
          "launch_rviz": 'false'
      }.items()
    )

    # Node start delay.
    delayed_start_picasso_arm = TimerAction(
      period=ur_Start_Delay,
      actions=[picasso_arm]
    )

    # Add the declare argument action
    ld = LaunchDescription()
    
    ld.add_action(declare_ur_type)
    ld.add_action(declare_ur_IP)
    ld.add_action(declare_ur_start_delay)
    ld.add_action(declare_ur_mock_hardware)
    ld.add_action(declare_rviz_config_file)
    ld.add_action(declare_launch_rviz)

    # Launch nodes
    ld.add_action(delayed_start_picasso_arm)
    ld.add_action(ur_sim_driver)
    ld.add_action(ur_driver)
    ld.add_action(rviz_node)
    
    ld.add_action(moveit)
    
    return ld