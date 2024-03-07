import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    obstacle_node = Node(
            package='obstacle_avoidance_tb3',
            executable='obstacle_avoidance.py',
            name='turtlebot_obstacle'
        )
    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch'),
                                      '/turtlebot3_dqn_stage2.launch.py'])
    )


    return LaunchDescription([
        gazebo_world,
        obstacle_node
    ])