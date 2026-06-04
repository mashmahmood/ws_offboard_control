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
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from ament_index_python.packages import get_package_share_path
from pathlib import Path

def generate_launch_description():
    # ---- Set global parameter ----
    set_sim_time = SetParameter(
        name='use_sim_time',
        value=ParameterValue(False, value_type=bool)
    )

    agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'serial', '--dev', '/dev/serial0', '-b', '921600'],
        output='log',
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(get_package_share_path('rplidar_ros') / 'launch' / 'rplidar_a1_launch.py')
    )

    positional_control_node = Node(
        package='px4_ros_com',
        executable='offboard_control_positional.py',
        name='px4_positional_control',
        output='screen',
    )

    camera_node = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[{
            'camera': '/base/soc/i2c0mux/i2c@1/ov5647@36',
            'width': 160,
            'height': 120,
            'format': 'YUYV'
        }]
    )

    # ---- URDF ----
    urdf_path = get_package_share_path('x500_description') / 'urdf' /'x500_tf.urdf'
    
    # 2. Read the actual raw XML contents of the URDF file
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # 3. Define the node with the parameters block
    urdf = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc  # Pass the XML text string here
        }]
    )
 
    px4_odom_converter_node = Node(
        package='px4_ros_com',
        executable='px4_odom_converter.py',
        name='odom_converter',
        parameters=[{'use_sim_time': False}],
        output='screen',
    )

    thermal = Node(
        package='thermal_cam',
        executable='thermal_pub',
        output='screen',
    )

    # ---~- SLAM Toolbox Launch ----
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(get_package_share_path('slam_toolbox') / 'launch' / 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': str(get_package_share_path('px4_ros_com') / 'config' / 'mapper_params_online_async.yaml'),
            'use_sim_time': False,
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
            'use_sim_time': False,
            'log_level': 'error',
        }.items(),
    )

    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        output='log',
        arguments=['--ros-args', '-p', 'port:=8765'],
        ros_arguments=['--log-level', 'error']
    )

    return LaunchDescription([
        set_sim_time,
        agent,
        camera_node,
        lidar,
        positional_control_node,
        urdf,
        px4_odom_converter_node,
        # slam_launch,
        slam_service,
        nav_launch,
        thermal,
        foxglove,
    ])
