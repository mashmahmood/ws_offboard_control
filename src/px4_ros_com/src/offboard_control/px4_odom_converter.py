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


class PX4OdomConverter(Node):

    def __init__(self):
        super().__init__('px4_odom_converter')

        # Subscriber
        self.sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.callback,
            qos_profile_sensor_data
        )

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    # -----------------------------
    # FRAME TRANSFORMS (from header)
    # -----------------------------

    def px4_to_ros_orientation(self, q_px4):
        """
        Correct conversion:
        PX4 gives: rotation from BODY (FRD) → WORLD (NED)

        We want: rotation from base_link (FLU) → WORLD (ENU)
        """

        # PX4 quaternion [w,x,y,z] → scipy [x,y,z,w]
        r = R.from_quat([q_px4[1], q_px4[2], q_px4[3], q_px4[0]])

        # Step 1: change world frame (NED → ENU)
        r_ned2enu = R.from_euler('z', np.pi/2) * R.from_euler('x', np.pi)

        # Step 2: change body frame (FRD → FLU)
        r_frd2flu = R.from_euler('x', np.pi)

        # ✅ CORRECT ORDER
        r_out = r_ned2enu * r * r_frd2flu

        q = r_out.as_quat()

        return np.array([q[3], q[0], q[1], q[2]], dtype=np.float32)

    def ned_to_enu_position(self, pos):
        """
        NED → ENU:
        x_e = y_n
        y_e = x_n
        z_e = -z_n
        """
        return np.array([pos[1], pos[0], -pos[2]])

    def ned_to_enu_velocity(self, vel):
        return np.array([vel[1], vel[0], -vel[2]])

    def frd_to_flu(self, vec):
        """
        FRD → FLU:
        x same
        y flip
        z flip
        """
        return np.array([vec[0], -vec[1], -vec[2]])

    # -----------------------------
    # CALLBACK
    # -----------------------------

    def callback(self, msg: VehicleOdometry):

        # --- Position ---
        pos_ned = np.array(msg.position)
        pos_enu = self.ned_to_enu_position(pos_ned)

        # --- Velocity ---
        vel_ned = np.array(msg.velocity)
        vel_enu = self.ned_to_enu_velocity(vel_ned)

        # --- Orientation ---
        q_px4 = msg.q
        q_ros = self.px4_to_ros_orientation(q_px4)

        # --- Angular velocity (FRD → FLU) ---
        ang_vel = self.frd_to_flu(np.array(msg.angular_velocity))

        # -----------------------------
        # Publish Odometry
        # -----------------------------
        odom = Odometry()

        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Position
        odom.pose.pose.position.x = float(pos_enu[0])
        odom.pose.pose.position.y = float(pos_enu[1])
        odom.pose.pose.position.z = float(pos_enu[2])

        # Orientation
        odom.pose.pose.orientation.w = float(q_ros[0])
        odom.pose.pose.orientation.x = float(q_ros[1])
        odom.pose.pose.orientation.y = float(q_ros[2])
        odom.pose.pose.orientation.z = float(q_ros[3])

        # Linear velocity
        odom.twist.twist.linear.x = float(vel_enu[0])
        odom.twist.twist.linear.y = float(vel_enu[1])
        odom.twist.twist.linear.z = float(vel_enu[2])

        # Angular velocity
        odom.twist.twist.angular.x = float(ang_vel[0])
        odom.twist.twist.angular.y = float(ang_vel[1])
        odom.twist.twist.angular.z = float(ang_vel[2])

        self.odom_pub.publish(odom)

        # -----------------------------
        # Publish TF (odom → base_link)
        # -----------------------------
        t = TransformStamped()

        t.header.stamp = odom.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = float(pos_enu[0])
        t.transform.translation.y = float(pos_enu[1])
        t.transform.translation.z = float(pos_enu[2])

        t.transform.rotation.w = float(q_ros[0])
        t.transform.rotation.x = float(q_ros[1])
        t.transform.rotation.y = float(q_ros[2])
        t.transform.rotation.z = float(q_ros[3])

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = PX4OdomConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()