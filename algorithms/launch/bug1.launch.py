import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    
    parameters = [{
        "obstacle_graph_resolution": 0.05,
        "path_resolution": 0.05,
        "maxSecurityDistance": 0.20,
        "x_size": 20.0,
        "y_size": 20.0,
        "output_file": '/home/momesso/navigation/src/algorithms/config/turtlebot3_house_with_boxes.bin',
    }]

  
    # rviz_config_file = os.path.join(get_package_share_directory('navigation_2d'), 'rviz', 'default.rviz')

      
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
            executable='load_all_points_to_cache',
            output='screen',
            parameters=parameters,
        ),

        # Node(
        #     package='algorithms',
        #     executable='publish_grid_map',
        #     output='screen',
        #     parameters=parameters,
        # ),

        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     arguments=['-d', rviz_config_file],
        # ),


    ])