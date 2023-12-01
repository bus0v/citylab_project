from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
        package = 'robot_patrol',
        executable = 'patrol_node',
        output = 'screen'),
        
    Node(
        package = 'robot_patrol',
        executable = 'start_direction_node',
        output = 'screen'),
        ])