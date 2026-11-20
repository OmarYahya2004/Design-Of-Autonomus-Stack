import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    lio_sam_share = get_package_share_directory('lio_sam')
    pkg_share = get_package_share_directory('csen934_project_pkg')
    
    # Dynamically locates the YAML file on any machine
    params_file = os.path.join(pkg_share, 'config', 'lio_sam_params.yaml')

    # 1. Static TF: base_link -> lidar
    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar_tf',
        arguments=['0', '0', '1.3', '0', '0', '0', 'base_link', 'lidar'],
        parameters=[{'use_sim_time': True}],
    )

    # 2. Include LIO-SAM's launch file with your custom params
    lio_sam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lio_sam_share, 'launch', 'run.launch.py')
        ),
        launch_arguments={'params_file': params_file}.items(),
    )

    return LaunchDescription([
        static_tf_lidar,
        lio_sam_launch,
    ])
