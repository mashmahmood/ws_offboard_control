#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanFrameFixer(Node):
    def __init__(self):
        super().__init__('scan_frame_fixer')
        self.sub = self.create_subscription(
            LaserScan,
            '/scan_raw',
            self.cb,
            10
        )
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.get_logger().info('Frame renamed.')

    def cb(self, msg):
        msg.header.frame_id = 'lidar_link'
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = ScanFrameFixer()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
