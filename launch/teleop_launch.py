import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    
    slam_arg = DeclareLaunchArgument('slam', default_value='toolbox',
                                 description='toolbox | gmapping | none')
                                 
    def slam_is(name):
        return IfCondition(PythonExpression(
            ["'", LaunchConfiguration('slam'), "' == '", name, "'"]))

    ui_node = Node(
        package='csen934_project_pkg',
        executable='control',
        name='control_teleop_node',
        output='screen'
    )
    
    noise_node = Node(
        package='csen934_project_pkg',
        executable='localization_add_noise',
        name='Localization_Add_Noise_Node',
        output='screen'
    )
    
    ekf_node = Node(
        package='csen934_project_pkg',
        executable='localization_ekf_node',
        name='Localization_EKF_Node',
        output='screen'
    )
    
    rqt_graph_node = Node(
        package='rqt_graph', executable='rqt_graph', name='rqt_graph'
    )

    yaw_node = Node(
        package='csen934_project_pkg',
        executable='yaw_plot_publisher',
        name='Yaw_Plot_Node',
        output='screen'
    )

    plots_node = Node(
        package='csen934_project_pkg',
        executable='localization_plots',
        name='Localization_Plots_Node',
        output='screen'
    )
    
    pc_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[
            ('cloud_in', '/lidar_points'),
            ('scan', '/scan')
        ],
        parameters=[{
            'target_frame': 'base_link', # Sliced relative to chassis
            'transform_tolerance': 0.2,
            'min_height': -0.5,           # Ignore ground hits
            'max_height': 3.0,           # Ignore roof
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,
            'scan_time': 0.1,
            'range_min': 0.45,
            'range_max': 50.0,
            'use_inf': True,
        }]
    )
    
    # Adjust the '1.3' Z-offset below if your LiDAR is mounted higher or lower on the Tesla roof
    lidar_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_tf_publisher',
        arguments=['0.0', '0.0', '1.3', '0.0', '0.0', '0.0', 'base_link', 'lidar']
    )

    slam_toolbox_node = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', output='screen',
        parameters=[os.path.join(get_package_share_directory('csen934_project_pkg'),
                                 'config', 'slam_toolbox.yaml')],
        arguments=[
	    '--ros-args',
	    '--log-level',
	    'debug'
	],
        condition=slam_is('toolbox')
    )

    gmapping_node = Node(
        package='slam_gmapping', executable='slam_gmapping',
        name='slam_gmapping', output='screen',
        parameters=[{'base_frame': 'base_link', 'odom_frame': 'odom',
                     'map_frame': 'map', 'map_update_interval': 2.0,
                     'maxUrange': 40.0, 'delta': 0.1}],
        remappings=[('scan', '/scan')],
        condition=slam_is('gmapping')
    )

    return LaunchDescription([
        slam_arg, ui_node, noise_node, ekf_node, rqt_graph_node, yaw_node,
        plots_node, pc_to_laserscan_node, lidar_tf_node,
        slam_toolbox_node, gmapping_node,
    ])
