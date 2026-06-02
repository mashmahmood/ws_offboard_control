from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_path
from pathlib import Path

def generate_launch_description():

    use_sim = LaunchConfiguration('use_sim')

    # ---- ARG ----
    declare_use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value='False',
        description='Run in simulation mode'
    )


    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(get_package_share_path('slam_toolbox') / 'launch' / 'online_async_launch.py')
        ),
        launch_arguments={
            'params_file': str(get_package_share_path('px4_ros_com') / 'config' / 'mapper_params_online_async.yaml'),
            'use_sim_time': use_sim,
        }.items(),
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(get_package_share_path('nav2_bringup') / 'launch' / 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': str(get_package_share_path('px4_ros_com') / 'config' / 'nav2_params.yaml'),
            'use_sim_time': use_sim,
        }.items()
    )

    slam_to_px4 = Node(
        package='px4_ros_com',
        executable='slam_to_px4.py',
        name='slam_to_px4',
        output='screen',
    )

    return LaunchDescription([
        declare_use_sim,
        slam_launch,
        nav_launch,
        slam_to_px4
    ])
