# Copyright 2023 Clearpath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('model', default_value='standard',
                          choices=['standard', 'lite'],
                          description='Turtlebot4 Model'),

    DeclareLaunchArgument('algo', default_value='fifo',
                          choices=['fifo', 'round_robin', 'rms', 'edf', 'lstf'],
                          description='Offload Server Scheduling Algorithm'),

    DeclareLaunchArgument('robot_id', default_value='turtlebot4_default', description='Turtlebot4 Default ID')
]


def generate_launch_description():

    # Directories
    pkg_offload_agent_gz_bringup = get_package_share_directory('offload_agent_gz_bringup')

    # Parameters
    agent_param_file_cmd = DeclareLaunchArgument(
        'agent_param_file',
        default_value=PathJoinSubstitution(
            [pkg_offload_agent_gz_bringup, 'config', 'offload_agent_node.yaml']),
        description='Offload Agent Turtlebot4 param file'
    )

    server_param_file_cmd = DeclareLaunchArgument(
        'server_param_file',
        default_value=PathJoinSubstitution(
            [pkg_offload_agent_gz_bringup, 'config', 'offload_server_node.yaml']),
        description='Offload Server param file'
    )

    offload_agent_node_yaml_file = LaunchConfiguration('agent_param_file')
    #offload_server_node_yaml_file = LaunchConfiguration('server_param_file')

    # Turtlebot4 node
    offload_agent_node = Node(
        package='offload_agent',
        name='offload_agent',
        executable='offload_agent',
        parameters=[offload_agent_node_yaml_file,
                    {'model': LaunchConfiguration('model')},
                    {'robot_id': LaunchConfiguration('robot_id')}],
        output='screen',
    )

    offload_server_node = Node(
        package='offload_server',
        name='offload_server',
        executable='offload_server',
        parameters=[{'algo': LaunchConfiguration('algo')}],
        output='screen',
    )

    # Turtlebot4 Gazebo Hmi node
    offload_agent_gz_hmi_node = Node(
        package='turtlebot4_gz_toolbox',
        name='turtlebot4_gz_hmi_node',
        executable='turtlebot4_gz_hmi_node',
        output='screen',
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('model'), 'standard'))
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(server_param_file_cmd)
    ld.add_action(agent_param_file_cmd)
    #ld.add_action(offload_server_node)
    ld.add_action(offload_agent_node)
    ld.add_action(offload_agent_gz_hmi_node)
    return ld
