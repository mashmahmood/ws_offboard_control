#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import subprocess
import os
from ament_index_python.packages import get_package_share_path

class NodeManager(Node):
    def __init__(self):
        super().__init__('web_node_manager')
        
        # A single service that handles both starting and stopping
        self.toggle_srv = self.create_service(Trigger, 'toggle_slam', self.toggle_slam_callback)
        
        self.slam_process = None

    def toggle_slam_callback(self, request, response):
        # Check if the process is NOT running (either never started, or started but stopped)
        if self.slam_process is None or self.slam_process.poll() is not None:
            self.get_logger().info('SLAM is idle. Starting SLAM Node via Foxglove...')
            
            try:
                # Dynamically look up the paths just like a launch file does
                slam_toolbox_launch = str(get_package_share_path('slam_toolbox') / 'launch' / 'online_async_launch.py')
                custom_params_file = str(get_package_share_path('px4_ros_com') / 'config' / 'mapper_params_online_async.yaml')
                
                # Construct the exact terminal command matching your launch requirements
                cmd = [
                    "ros2", "launch", 
                    slam_toolbox_launch,
                    f"slam_params_file:={custom_params_file}",
                ]
                
                # Execute the launch command in the background
                self.slam_process = subprocess.Popen(cmd)
                
                response.success = True
                response.message = "SLAM successfully launched!"
                
            except Exception as e:
                response.success = False
                response.message = f"Failed to launch SLAM: {str(e)}"
                
        else:
            # If the process .poll() returns None, it means it's actively running. Time to stop it.
            self.get_logger().info('SLAM is active. Stopping SLAM Node via Foxglove...')
            
            self.slam_process.terminate()
            self.slam_process.wait() # Wait for it to clean up fully
            self.slam_process = None # Reset the process tracker
            
            response.success = True
            response.message = "SLAM stopped successfully."
            
        return response

def main(args=None):
    rclpy.init(args=args)
    node = NodeManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()