import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('sdsl_ros2')
    
    # Declare map file argument
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml_file',
        default_value=os.path.join(pkg_dir, 'config', 'my_map.yaml'),
        description='Full path to map yaml file to load'
    )
    
    # Map server node (publishes on /map_original)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{
            'use_sim_time': False,
            'yaml_filename': LaunchConfiguration('map_yaml_file'),
            'frame_id': 'map'
        }],
        remappings=[('map', 'map_original')],
        output='screen'
    )
    
    # Configure the lifecycle node
    configure_map_server = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/map_server', 'configure'],
        output='screen'
    )
    
    # Activate the lifecycle node
    activate_map_server = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/map_server', 'activate'],
        output='screen'
    )
    
    # Republisher node for periodic publishing
    map_republisher_node = Node(
        package='sdsl_ros2',
        executable='map_republisher_node',
        name='map_republisher',
        output='screen'
    )
    
    return LaunchDescription([
        map_yaml_arg,
        map_server_node,
        TimerAction(
            period=2.0,  # Wait 2 seconds for node to start
            actions=[configure_map_server]
        ),
        TimerAction(
            period=4.0,  # Wait 4 seconds total before activating
            actions=[activate_map_server]
        ),
        TimerAction(
            period=5.0,  # Start republisher after map_server is active
            actions=[map_republisher_node]
        )
    ])