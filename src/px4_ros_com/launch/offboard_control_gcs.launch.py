#!/usr/bin/env python

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition

from ament_index_python.packages import get_package_share_path
from pathlib import Path

def generate_launch_description():
    # ---- RViz ----
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d', str(get_package_share_path('nav2_bringup') / 'rviz' / 'nav2_default_view.rviz')
        ],
        output='screen',
    )

    temp_node = Node(
        package='rviz_2d_overlay_plugins',
        executable='string_to_overlay_text',
        name='string_to_overlay_text_1',
        output='screen',
        parameters=[
            {"string_topic": "thermal/max_temp"},
            {"fg_color": "b"}, # colors can be: r,g,b,w,k,p,y (red,green,blue,white,black,pink,yellow)
        ],
    )
    # ---- Keyboard Control Node ----
    control_keyboard_node = Node(
        package='px4_ros_com',
        executable='control_keyboard.py',
        prefix='gnome-terminal --',
        output='screen',
    )

    return LaunchDescription([
        rviz_node,
        temp_node,
        # control_keyboard_node,
    ])