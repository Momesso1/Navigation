import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config_dir = os.path.join(
        get_package_share_directory('algorithms'),
        'config'
    )

    rviz_dir = os.path.join(
        get_package_share_directory('algorithms'),
        'rviz'
    )

    pose_file = os.path.join(config_dir, 'locations.yaml')
    map_filename = os.path.join(config_dir, 'occupancy_map.yaml')
    map_pgm_filename = os.path.join(config_dir, 'occupancy_map.png')

    rviz_file = os.path.join(rviz_dir, "d_star_and_rrt.rviz")

    parameters = [{
        "obstacle_graph_resolution": 0.05,
        "path_resolution": 0.05,
        "maxSecurityDistance": 0.30,
        "x_upper_bound": 3.0,
        "x_lower_bound": -17.0,
        "y_upper_bound": 18.0,
        "y_lower_bound": -2.0,
        "pose_file": pose_file,
        "map_yaml_file": map_filename,
        "map_image_file": map_pgm_filename
    }]

    return LaunchDescription([

        # Node(
        #     package='algorithms',
        #     executable='bug2',
        #     output='screen',
        #     parameters=parameters,
        # ),

        Node(
            package='algorithms',
            executable='rrt',
            parameters=parameters,
            output='screen',
        ),

        Node(
            package='algorithms',
            executable='controller',
            parameters=parameters,
            output='screen',
        ),

        Node(
            package='algorithms',
            executable='send_poses',
            parameters=parameters,
            output='screen',
        ),


        Node(
            package='algorithms',
            executable='create_obstacle_graph_with_occupancy_map',
            output='screen',
            parameters=parameters,
        ),

        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     arguments=['-d', rviz_file],
        # ),
    ])
