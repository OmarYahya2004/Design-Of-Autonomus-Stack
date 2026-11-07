import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Launch your custom UI Teleop node
    ui_node = Node(
        package='csen934_project_pkg',
        executable='ui_teleop',
        name='control_teleop_node',
        output='screen'
    )
    
    # Launch the Noise Addition node
    #noise_node = Node(
    #    package='csen934_project_pkg',
    #    executable='localization_add_noise',
    #    name='localization_add_noise',
    #    output='screen'
    #)
    
    # Launch the EKF node
    #ekf_node = Node(
    #    package='csen934_project_pkg',
    #    executable='localization_ekf_node',
    #    name='Localization_EKF_Node',
    #    output='screen'
    #)
    
    rqt_graph_node = Node(
        package='rqt_graph',
        executable='rqt_graph',
        name='rqt_graph'
    )

    return LaunchDescription([
        ui_node,
        #noise_node,
        #ekf_node,
        rqt_graph_node
    ])
