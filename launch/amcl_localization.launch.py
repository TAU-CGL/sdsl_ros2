import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('sdsl_ros2')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    initial_pose_x_arg = DeclareLaunchArgument(
        'initial_pose_x',
        default_value='0.0',
        description='Initial pose x coordinate'
    )

    initial_pose_y_arg = DeclareLaunchArgument(
        'initial_pose_y',
        default_value='0.0',
        description='Initial pose y coordinate'
    )

    initial_pose_yaw_arg = DeclareLaunchArgument(
        'initial_pose_yaw',
        default_value='0.0',
        description='Initial pose yaw angle'
    )

    amcl_params = {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'map_topic': 'map',
        'scan_topic': 'sds',
        'alpha1': 0.2,
        'alpha2': 0.2,
        'alpha3': 0.2,
        'alpha4': 0.2,
        'alpha5': 0.2,
        'base_frame_id': 'base_footprint',
        'beam_skip_distance': 0.5,
        'beam_skip_error_threshold': 0.9,
        'beam_skip_threshold': 0.3,
        'do_beamskip': False,
        'global_frame_id': 'map',
        'lambda_short': 0.1,
        'laser_likelihood_max_dist': 2.0,
        'laser_max_range': 100.0,
        'laser_min_range': -1.0,
        'laser_model_type': 'likelihood_field',
        'max_beams': 60,
        'max_particles': 2000,
        'min_particles': 500,
        'odom_frame_id': 'odom',
        'pf_err': 0.05,
        'pf_z': 0.99,
        'recovery_alpha_fast': 0.0,
        'recovery_alpha_slow': 0.0,
        'resample_interval': 1,
        'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
        'save_pose_rate': 0.5,
        'sigma_hit': 0.2,
        'tf_broadcast': True,
        'transform_tolerance': 1.0,
        'update_min_a': 0.2,
        'update_min_d': 0.25,
        'z_hit': 0.5,
        'z_max': 0.05,
        'z_rand': 0.5,
        'z_short': 0.05,
        'allow_parameter_qos_overrides': True,
        'initial_pose': {
            'x': LaunchConfiguration('initial_pose_x'),
            'y': LaunchConfiguration('initial_pose_y'),
            'z': 0.0,
            'yaw': LaunchConfiguration('initial_pose_yaw')
        }
    }

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[
            amcl_params,
            {
                'qos_overrides./map.subscriber.durability': 'volatile',
                'qos_overrides./map.subscriber.reliability': 'reliable'
            }
        ],
        output='screen'
    )

    configure_amcl = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/amcl', 'configure'],
        output='screen'
    )

    activate_amcl = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/amcl', 'activate'],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        initial_pose_x_arg,
        initial_pose_y_arg,
        initial_pose_yaw_arg,
        amcl_node,
        TimerAction(
            period=2.0,
            actions=[configure_amcl]
        ),
        TimerAction(
            period=4.0,
            actions=[activate_amcl]
        )
    ])