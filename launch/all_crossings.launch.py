import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('map_publisher'),
        'rviz', 'config.rviz'
    )

    return LaunchDescription([

        Node(
            package='map_publisher',
            executable='lanelet_visualizer',
            name='lanelet_visualizer',
        ),

        Node(
            package='map_publisher',
            executable='object_publisher',
            name='object_publisher_c1',
            remappings=[('/object_markers', '/object_markers_c1')],
            parameters=[{
                'map_offset_x': 88.0,
                'map_offset_y': -234.0,
                'object_list_path': '/home/rahim/ros2_ws/src/map_publisher/map_publisher/20241126_0017_crossing1_00.json',
            }]
        ),

        Node(
            package='map_publisher',
            executable='object_publisher',
            name='object_publisher_c2',
            remappings=[('/object_markers', '/object_markers_c2')],
            parameters=[{
                'map_offset_x': -183.04,
                'map_offset_y': -322.9,
                'object_list_path': '/mnt/c/Users/ibrah/Downloads/urbaning_labels/20241126_0001_crossing2_00.json',
            }]
        ),

        Node(
            package='map_publisher',
            executable='object_publisher',
            name='object_publisher_c3',
            remappings=[('/object_markers', '/object_markers_c3')],
            parameters=[{
                'map_offset_x': 43.41,
                'map_offset_y': 62.88,
                'object_list_path': '/mnt/c/Users/ibrah/Downloads/urbaning_labels/20241127_0010_crossing3_08.json',
            }]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
        ),

    ])
