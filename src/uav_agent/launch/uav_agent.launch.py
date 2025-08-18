#!/usr/bin/env python3
#  Copyright (c) 2024. Jet Propulsion Laboratory. All rights reserved.
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#  https://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for UAV agent."""
    
    # Declare launch arguments
    streaming_arg = DeclareLaunchArgument(
        'streaming',
        default_value='true',
        description='Enable streaming mode for real-time response display'
    )
    
    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Enable verbose output for debugging'
    )
    
    # UAV Agent node
    uav_agent_node = Node(
        package='uav_agent',
        executable='uav_agent.py',
        name='rosa_uav_agent',
        output='screen',
        parameters=[{
            'streaming': LaunchConfiguration('streaming'),
            'verbose': LaunchConfiguration('verbose'),
        }],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        streaming_arg,
        verbose_arg,
        uav_agent_node,
    ])
