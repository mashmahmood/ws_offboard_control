#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation as R

def ned_to_enu_position(pos_ned):
    """Convert position vector from NED to ENU frame."""
    x_enu = pos_ned[1]
    y_enu = pos_ned[0]
    z_enu = -pos_ned[2]
    return np.array([x_enu, y_enu, z_enu])

def aircraft_to_baselink_angular_velocity(ang_vel):
    """Rotate angular velocity vector from aircraft to base_link frame."""
    rot = R.from_euler('x', np.pi)
    return rot.apply(ang_vel)

def px4_to_ros_orientation(quat_px4):
    """
    Convert PX4 quaternion (aircraft->NED) to ROS quaternion (base_link->ENU).
    Step 1: NED->ENU rotation (+90 deg Z, +180 deg X)
    Step 2: Aircraft->Base_link rotation (+180 deg X)
    """
    # Convert PX4 [w,x,y,z] to scipy [x,y,z,w]
    r_px4 = R.from_quat([quat_px4[1], quat_px4[2], quat_px4[3], quat_px4[0]])

    # NED->ENU rotation: +90 deg Z, +180 deg X
    r_ned_enu = R.from_euler('z', np.pi/2) * R.from_euler('x', np.pi)

    # Aircraft->Base_link rotation: +180 deg X
    r_aircraft_baselink = R.from_euler('x', np.pi)

    r_ros = r_aircraft_baselink * r_ned_enu * r_px4
    roll, pitch, yaw = r_ros.as_euler('xyz')
    
    # TO handle the FRD -> FLU
    yaw = -yaw  # flip yaw direction	
    pitch = -pitch # flip pitch

    r_fixed = R.from_euler('xyz', [roll, pitch, yaw])

    # Convert back to quaternion (x,y,z,w)
    q_ros = r_fixed.as_quat()

    # Convert back to [w,x,y,z]
    return np.array([q_ros[3], q_ros[0], q_ros[1], q_ros[2]])

class PX4OdomConverterNode(Node):
    def __init__(self):
        super().__init__('px4_odom_converter')
        
        
        odom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odom_callback,
            qos_profile_sensor_data)
        self.pub = self.create_publisher(Odometry, 'odom', odom_qos)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def odom_callback(self, msg: VehicleOdometry):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Position
        pos_ned = np.array(msg.position)
        pos_enu = ned_to_enu_position(pos_ned)
        odom_msg.pose.pose.position.x = float(pos_enu[0])
        odom_msg.pose.pose.position.y = float(pos_enu[1])
        odom_msg.pose.pose.position.z = float(pos_enu[2])

        # Orientation
        quat_px4 = np.array(msg.q)  # [w,x,y,z]
        quat_ros = px4_to_ros_orientation(quat_px4)
        odom_msg.pose.pose.orientation.w = float(quat_ros[0])
        odom_msg.pose.pose.orientation.x = float(quat_ros[1])
        odom_msg.pose.pose.orientation.y = float(quat_ros[2])
        odom_msg.pose.pose.orientation.z = float(quat_ros[3])

        # Linear velocity
        vel_ned = np.array(msg.velocity)
        vel_enu = ned_to_enu_position(vel_ned)  # same transform as position
        odom_msg.twist.twist.linear.x = float(vel_enu[0])
        odom_msg.twist.twist.linear.y = float(vel_enu[1])
        odom_msg.twist.twist.linear.z = float(vel_enu[2])

        # Angular velocity
        ang_vel_aircraft = np.array(msg.angular_velocity)
        #ang_vel_baselink = aircraft_to_baselink_angular_velocity(ang_vel_aircraft)
        ang_vel_baselink = ang_vel_aircraft
        odom_msg.twist.twist.angular.x = float(ang_vel_baselink[0])
        odom_msg.twist.twist.angular.y = float(ang_vel_baselink[1])
        odom_msg.twist.twist.angular.z = float(ang_vel_baselink[2])


        odom_msg.pose.covariance = [0.0] * 36

        # Position → more trusted now (reduces jitter)
        odom_msg.pose.covariance[0] = 0.03   # x
        odom_msg.pose.covariance[7] = 0.03   # y
        odom_msg.pose.covariance[14] = 0.08  # z (still less reliable)

        # Orientation → strongly trusted (critical for 2D SLAM stability)
        odom_msg.pose.covariance[21] = 0.02  # roll
        odom_msg.pose.covariance[28] = 0.02  # pitch
        odom_msg.pose.covariance[35] = 0.02  # yaw

        odom_msg.twist.covariance = [0.0] * 36

        # Linear velocity → slightly trusted
        odom_msg.twist.covariance[0] = 0.05
        odom_msg.twist.covariance[7] = 0.05
        odom_msg.twist.covariance[14] = 0.1

        # Angular velocity → more trusted for smooth rotation
        odom_msg.twist.covariance[21] = 0.03
        odom_msg.twist.covariance[28] = 0.03
        odom_msg.twist.covariance[35] = 0.04


        self.pub.publish(odom_msg)
        
        # Also broadcast tf odom -> base_link
        t = TransformStamped()
        t.header.stamp = odom_msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = float(odom_msg.pose.pose.position.x)
        t.transform.translation.y = float(odom_msg.pose.pose.position.y)
        t.transform.translation.z = float(odom_msg.pose.pose.position.z)
        t.transform.rotation = odom_msg.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = PX4OdomConverterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

