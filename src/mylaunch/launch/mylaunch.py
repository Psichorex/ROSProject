from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable

def generate_launch_description():
    return LaunchDescription([

        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),

        ExecuteProcess(
            cmd=['ros2', 'launch', 'turtlebot3_gazebo', 'turtlebot3_world.launch.py'],
            output='screen'),

        ExecuteProcess(
            cmd=['ros2', 'launch', 'turtlebot3_cartographer', 'cartographer.launch.py', 'use_sim_time:=True'],
            output='screen'),

        Node(
            package='my_teleop',
            executable='teleop',
            output='screen'),

        Node(
            package='sensordatareader',
            executable='sensordatareader',
            output='screen'),
    ])

