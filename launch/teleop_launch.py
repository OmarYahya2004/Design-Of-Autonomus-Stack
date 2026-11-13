import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Launch your custom UI Teleop node
    ui_node = Node(
        package='csen934_project_pkg',
        executable='control',
        name='control_teleop_node',
        output='screen'
    )
    
    # Launch the Noise Addition node
    noise_node = Node(
        package='csen934_project_pkg',
        executable='localization_add_noise',
        name='Localization_Add_Noise_Node',
        output='screen'
    )
    
    # Launch the EKF node
    ekf_node = Node(
        package='csen934_project_pkg',
        executable='localization_ekf_node',
        name='Localization_EKF_Node',
        output='screen'
    )
    
    rqt_graph_node = Node(
        package='rqt_graph',
        executable='rqt_graph',
        name='rqt_graph'
    )

    # Republishes yaw (deg, unwrapped) of GT / Noisy / Filtered for rqt_plot
    yaw_node = Node(
        package='csen934_project_pkg',
        executable='yaw_plot_publisher',
        name='Yaw_Plot_Node',
        output='screen'
    )

    # Three rqt_plot windows (x / y / yaw) comparing GT vs Noisy vs Filtered.
    # Waits for the topics first (rqt_plot drops topics that don't exist yet).
    plots_node = Node(
        package='csen934_project_pkg',
        executable='localization_plots',
        name='Localization_Plots_Node',
        output='screen'
    )

    return LaunchDescription([
        ui_node,
        noise_node,
        ekf_node,
        rqt_graph_node,
        yaw_node,
        plots_node
    ])
