import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Import the specific overlay message type
from rviz_2d_overlay_msgs.msg import OverlayText

import board
import busio
import adafruit_mlx90640
import numpy as np
import cv2


class ThermalPublisher(Node):

    def __init__(self):
        super().__init__('thermal_publisher')

        self.temp_pub = self.create_publisher(Image, 'thermal/temperature', 10)
        self.rgb_pub = self.create_publisher(Image, 'thermal/rgb', 10)
        
        # 1. Create the overlay publisher
        self.max_temp_pub = self.create_publisher(OverlayText, 'thermal/max_temp', 10)

        self.bridge = CvBridge()

        # Stable I2C
        i2c = busio.I2C(board.SCL, board.SDA, frequency=1000000)

        self.mlx = adafruit_mlx90640.MLX90640(i2c)
        self.mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_2_HZ

        self.frame = np.zeros((24 * 32,))
        self.timer = self.create_timer(0.5, self.timer_callback)  # 2 Hz

        self.get_logger().info("Thermal camera node started")

    def read_frame_with_retry(self, retries=10):
        for _ in range(retries):
            try:
                self.mlx.getFrame(self.frame)
                return True
            except:
                continue
        return False

    def timer_callback(self):

        if not self.read_frame_with_retry():
            self.get_logger().warn("Frame read failed after retries")
            return

        temp_array = np.reshape(self.frame, (24, 32)).astype(np.float32)

        # -------------------------
        # Publish raw temperature
        # -------------------------
        temp_msg = self.bridge.cv2_to_imgmsg(temp_array, encoding="32FC1")
        temp_msg.header.stamp = self.get_clock().now().to_msg()
        self.temp_pub.publish(temp_msg)

        # -------------------------
        # Interpolation (4x upscale)
        # -------------------------
        upscaled = cv2.resize(
            temp_array,
            (128, 96),  # width, height
            interpolation=cv2.INTER_CUBIC
        )

        # Normalize for visualization
        norm = cv2.normalize(upscaled, None, 0, 255, cv2.NORM_MINMAX)
        norm = np.uint8(norm)

        # Apply colormap
        rgb = cv2.applyColorMap(norm, cv2.COLORMAP_JET)

        rgb_msg = self.bridge.cv2_to_imgmsg(rgb, encoding="bgr8")
        rgb_msg.header.stamp = temp_msg.header.stamp
        self.rgb_pub.publish(rgb_msg)

        # -------------------------
        # Calculate & Publish Max Temp Overlay
        # -------------------------
        max_temp = np.max(temp_array)
        
        # 2. Build the OverlayText structure
        overlay_msg = OverlayText()
        overlay_msg.text = f"Max Temp: {max_temp:.1f}°C"
        
        # # Optional: Set baseline styling (can also be configured inside RViz display options)
        # overlay_msg.text_size = 14.0
        # overlay_msg.line_width = 2
        # overlay_msg.fg_color.r = 1.0  # Red text
        # overlay_msg.fg_color.g = 0.2
        # overlay_msg.fg_color.b = 0.2
        # overlay_msg.fg_color.a = 1.0  # Fully opaque
        
        # Publish the text overlay
        self.max_temp_pub.publish(overlay_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ThermalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
