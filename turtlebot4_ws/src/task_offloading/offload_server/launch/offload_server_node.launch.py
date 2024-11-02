import launch
import launch_ros.actions


ARGUMENTS = [
    DeclareLaunchArgument('algo', default_value='fifo',
                          choices=['fifo', 'round robin', 'rms', 'edf', 'lstf'],
                          description='Offloading Server')
]

def generate_launch_description():


    # Directories
    pkg_offload_server = get_package_share_directory('offload_server')

    # offloading server node
    turtlebot4_node = Node(
        package='offload_server',
        name='offload_server_node',
        executable='offload_server_node',
        output='screen',
    )
