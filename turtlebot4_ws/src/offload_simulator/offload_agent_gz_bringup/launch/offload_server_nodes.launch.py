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
    #offload_server_node_yaml_file = LaunchConfiguration('server_param_file')

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

    # Define LaunchDescription variable
    #sleep(60)
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(offload_server_node)
    return ld
