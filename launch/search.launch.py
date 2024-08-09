from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    search_node = Node(
        package='search_pkg',
        executable='search_node',
        parameters=[
                {'grid_count': 100},
                {'observation_radius': 3.0},
                {'observation_certainty': 1.0},
                {'n_sample': 7}
            ]
    )

    search_test = Node(
        package='search_pkg',
        executable='search_test',
    )


    return LaunchDescription([
        search_node,
        search_test
    ])
   


