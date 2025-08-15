from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'streaming',
            default_value='false',
            description='Enable streaming mode for LLM responses'
        ),
        
        DeclareLaunchArgument(
            'connection_string',
            default_value='udp://:14540',
            description='MAVSDK connection string for drone'
        ),
        
        Node(
            package='drone_agent',
            executable='drone_agent.py',
            name='rosa_drone_agent',
            output='screen',
            parameters=[
                {'streaming': LaunchConfiguration('streaming')},
                {'connection_string': LaunchConfiguration('connection_string')}
            ]
        )
    ])