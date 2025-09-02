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

    output_file = os.path.join(config_dir, 'turtlebot3_house_with_boxes.bin')
    pose_file = os.path.join(config_dir, 'locations.yaml')
    map_filename = os.path.join(config_dir, 'map.yaml')
    map_pgm_filename = os.path.join(config_dir, 'map.pgm')

    rviz_file = os.path.join(rviz_dir, "bug1.rviz")

    parameters = [{
        "obstacle_graph_resolution": 0.05,
        "path_resolution": 0.05,
        "maxSecurityDistance": 0.20,
        "x_size": 20.0,
        "y_size": 20.0,
        "output_file": output_file,
        "pose_file": pose_file,
        "map_filename": map_filename,
        "map_pgm_filename": map_pgm_filename
    }]

    return LaunchDescription([

        Node(
            package='algorithms',
            executable='bug1',
            output='screen',
            parameters=parameters,
        ),

        Node(
            package='algorithms',
            executable='send_poses',
            parameters=parameters,
            output='screen',
        ),

        Node(
            package='algorithms',
            executable='load_all_points_to_cache',
            output='screen',
            parameters=parameters,
        ),

        # Node(
        #     package='algorithms',
        #     executable='create_graph',
        #     output='screen',
        #     parameters=parameters,
        # ),

        # Node(
        #     package='algorithms',
        #     executable='create_graph_with_chuncks',
        #     output='screen',
        #     parameters=parameters,
        # ),

        Node(
            package='algorithms',
            executable='publish_grid_map',
            output='screen',
            parameters=parameters,
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_file],
        ),
    ])
