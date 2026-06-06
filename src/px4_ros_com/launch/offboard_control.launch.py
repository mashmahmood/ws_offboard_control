#!/usr/bin/env python

################################################################################
#
# Copyright (c) 2018-2022, PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
# may be used to endorse or promote products derived from this software without
# specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
################################################################################


from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import ExecuteProcess, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    # ---- Set global parameter ----
    set_sim_time = SetParameter(
        name='use_sim_time',
        value=ParameterValue(True, value_type=bool)
    )

    processes_node = Node(
        package='px4_ros_com',
        executable='processes.py',
        name='processes',
        prefix='gnome-terminal --'
    )
    positional_control_node = Node(
        package='px4_ros_com',
        executable='offboard_control_positional.py',
        name='px4_positional_control'
    )
    gazebo_to_ros_node = Node(
    	package='ros_gz_bridge',
    	executable='parameter_bridge',
    	arguments=[
    	    # LaserScan bridge
    	    # '/world/turtle_world/model/x500_lidar_2d_0/link/lidar_link/sensor/lidar_2d_v2/scan'
    	    '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
    	    
    	    # Clock bridge (ADD THIS)
    	    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',],
	# remappings=[('/world/turtle_world/model/x500_lidar_2d_0/link/lidar_link/sensor/lidar_2d_v2/scan', '/scan_raw')],
        output='screen',
    )

    urdf = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[str(get_package_share_path('x500_description') / 'urdf' / 'x500_tf.urdf')],
        output='screen',
    )

    px4_odom_converter_node = Node(
        package='px4_ros_com',
        executable='px4_odom_converter.py',
        name='odom_converter',
        parameters=[{'use_sim_time': True}]
    )
    # scan_frame_fixer_node = Node(
    #     package='px4_ros_com',
    #     executable='scan_frame_fixer.py',
    #     name='scan_frame_fixer',
    #     parameters=[{'use_sim_time': True}]
    # )

    # ---~- SLAM Toolbox Launch ----
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(get_package_share_path('slam_toolbox') / 'launch' / 'online_async_launch.py')
        ),
        launch_arguments={
            'params_file': str(get_package_share_path('px4_ros_com') / 'config' / 'mapper_params_online_async.yaml'),
            'use_sim_time': 'true',
        }.items(),
    )


    slam_service = Node(
        package='px4_ros_com',
        executable='slam_service.py',
        output='screen',
    )

    # ---- Nav2 Launch ----
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(get_package_share_path('nav2_bringup') / 'launch' / 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': str(get_package_share_path('px4_ros_com') / 'config' / 'nav2_params.yaml'),
            'use_sim_time': 'true',
            'log_level': 'error',
        }.items(),
    )    


    # foxglove = Node(
    #     package='foxglove_bridge',
    #     executable='foxglove_bridge',
    #     output='log',
    #     arguments=['--ros-args', '-p', 'port:=8765'],
    #     ros_arguments=['--log-level', 'error']
    # )
    # foxglove = Node(
    #     package='foxglove_bridge',
    #     executable='foxglove_bridge_node',
    #     name='foxglove_bridge',
    #     # arguments=['--ros-args', '--log-level', 'error'],
    #     output='log',
    #     parameters=[{
    #         'port': 8765,  # Replace with youxr desired custom port
    #         # 'address': '0.0.0.0' # Allows connections outside localhost
    #     }]
    # )

    return LaunchDescription([
        set_sim_time,
        processes_node,
        positional_control_node,
        urdf,
        px4_odom_converter_node,
        # scan_frame_fixer_node,
        gazebo_to_ros_node,
        # slam_launch,
        slam_service,
        nav_launch,
        #foxglove,
    ])
