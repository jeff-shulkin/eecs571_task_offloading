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
from time import sleep
from ament_index_python.packages import get_package_share_directory
from irobot_create_common_bringup.namespace import GetNamespacedName

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('algo', default_value='fifo',
                          choices=['fifo', 'round_robin', 'rms', 'edf', 'lstf'],
                          description='Offload Server Scheduling Algorithm'),
]


def generate_launch_description():

    # Directories
    pkg_offload_server_gz_bringup = get_package_share_directory('offload_agent_gz_bringup')

    # Parameters
    robot_name = GetNamespacedName('offload_server', 'offload_agent')

    # Offload Server Node
    offload_server_node = Node(
        package='offload_server',
        name='offload_server',
        executable='offload_server',
        parameters=[{'algo': LaunchConfiguration('algo')}],
        remappings=[('/offload_server/offload_localization/_action/feedback', '/offload_localization/_action/feedback'),
                    ('/offload_server/offload_localization/_action/status', '/offload_localization/_action/status'),
                    ('/offload_server/offload_localization/_action/cancel_goal', '/offload_localization/_action/cancel_goal'),
                    ('/offload_server/offload_localization/_action/get_result', '/offload_localization/_action/get_result'),
                    ('/offload_server/offload_localization/_action/send_goal', '/offload_localization/_action/send_goal')
                   ],
        output='screen',
    )

    static_map_odom_transform_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_odom_tf_publisher',
            arguments=['0.0', '0.0', '0.0', '0', '0', '0', 'map', 'odom'],
            remappings=[('/tf_static', '/offload_server/tf_static')],
            output='screen',
    )

    static_odom_base_transform_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_odom_base_tf_publisher',
            arguments=['0.0', '0.0', '0.0', '0', '0', '0', 'odom', 'base_link'],
            remappings=[('/tf_static', '/offload_server/tf_static')],
            output='screen',
    )


    static_base_rplidar_transform_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_base_rplidar_tf_publisher',
            arguments=['0.0', '0.0', '0.0', '0', '0', '0', 'base_link', 'rplidar_link'],
            remappings=[('/tf_static', '/offload_server/tf_static')],
            output='screen',
    )

    # RPLIDAR static transforms
    static_rplidar_transform_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_rplidar_tf_publisher',
            output='screen',
            arguments=[
                '0', '0', '0', '0', '0', '0.0',
                'rplidar_link', [robot_name, '/rplidar_link/rplidar']],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ]
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(offload_server_node)
    ld.add_action(static_map_odom_transform_node)
    ld.add_action(static_odom_base_transform_node)
    ld.add_action(static_base_rplidar_transform_node)
    ld.add_action(static_rplidar_transform_node)
    return ld
