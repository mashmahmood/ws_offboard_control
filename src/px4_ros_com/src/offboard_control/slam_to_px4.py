#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import tf2_ros
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import TransformStamped

import numpy as np
from scipy.spatial.transform import Rotation as R


class SlamToPx4Bridge(Node):

    def __init__(self):
        super().__init__('slam_to_px4_bridge')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)  # ✅ NEW

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.publisher_ = self.create_publisher(
            VehicleOdometry,
            '/fmu/in/vehicle_visual_odometry',
            qos_profile
        )

        self.timer = self.create_timer(0.033, self.timer_callback)

    # -----------------------------
    # ROS → PX4
    # -----------------------------
    def ros_to_px4_orientation(self, q_ros):
        r = R.from_quat([q_ros.x, q_ros.y, q_ros.z, q_ros.w])

        r_enu2ned = R.from_euler('x', np.pi) * R.from_euler('z', -np.pi/2)
        r_flu2frd = R.from_euler('x', np.pi)

        r_out = r_enu2ned * r * r_flu2frd
        q = r_out.as_quat()

        return [float(q[3]), float(q[0]), float(q[1]), float(q[2])]

    def enu_to_ned_position(self, t):
        return [float(t.y), float(t.x), float(-t.z)]

    # -----------------------------
    # PX4 → ROS (for debug)
    # -----------------------------
    def px4_to_ros_orientation(self, q_px4):
        r = R.from_quat([q_px4[1], q_px4[2], q_px4[3], q_px4[0]])

        r_ned2enu = R.from_euler('z', np.pi/2) * R.from_euler('x', np.pi)
        r_frd2flu = R.from_euler('x', np.pi)

        r_out = r_ned2enu * r * r_frd2flu
        q = r_out.as_quat()

        return [q[0], q[1], q[2], q[3]]  # x,y,z,w

    def ned_to_enu_position(self, pos):
        return [pos[1], pos[0], -pos[2]]

    # -----------------------------
    def timer_callback(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )

            msg = VehicleOdometry()

            now = self.get_clock().now().nanoseconds // 1000
            msg.timestamp = int(now)
            msg.timestamp_sample = int(now)

            # --- POSITION ---
            msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
            msg.position = self.enu_to_ned_position(t.transform.translation)

            # --- ORIENTATION ---
            msg.q = self.ros_to_px4_orientation(t.transform.rotation)

            # --- OPTIONAL ---
            msg.velocity = [float('nan')] * 3
            msg.angular_velocity = [float('nan')] * 3

            msg.position_variance = [0.001, 0.001, 0.002]
            msg.orientation_variance = [0.01, 0.01, 0.02]

            # ✅ Publish to PX4
            self.publisher_.publish(msg)

            # =====================================
            # ✅ DEBUG TF: PX4 → ROS (visualize loop)
            # =====================================

            pos_enu = self.ned_to_enu_position(msg.position)
            q_enu = self.px4_to_ros_orientation(msg.q)

            tf_msg = TransformStamped()
            tf_msg.header.stamp = self.get_clock().now().to_msg()
            tf_msg.header.frame_id = "map"
            tf_msg.child_frame_id = "px4_debug"

            tf_msg.transform.translation.x = float(pos_enu[0])
            tf_msg.transform.translation.y = float(pos_enu[1])
            tf_msg.transform.translation.z = float(pos_enu[2])

            tf_msg.transform.rotation.x = float(q_enu[0])
            tf_msg.transform.rotation.y = float(q_enu[1])
            tf_msg.transform.rotation.z = float(q_enu[2])
            tf_msg.transform.rotation.w = float(q_enu[3])

            self.tf_broadcaster.sendTransform(tf_msg)

        except Exception as e:
            self.get_logger().warn(f'Could not transform: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = SlamToPx4Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()