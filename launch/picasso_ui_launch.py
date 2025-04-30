#!/usr/bin/env python3

from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
  ld = LaunchDescription()
 
  # nodes
  picasso_ui = Node(
    package="picasso_bot",
    executable="picasso_ui",
    name="node_picasso_ui",
    output="screen",
    emulate_tty=True
  )

  # add actions to launch
  ld.add_action(picasso_ui)

  return ld

