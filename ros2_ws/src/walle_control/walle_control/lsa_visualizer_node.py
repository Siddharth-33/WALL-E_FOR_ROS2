#!/usr/bin/env python3
"""
lsa_visualizer_node.py
======================
Independent LSA (Line Sensor Array) visualizer for Wall-E.
Works in ALL modes (including IDLE mode 0) — you can pick up the robot
and test black vs white surface contrast with your hands.

FEATURES:
  1. RViz 3D High-Contrast Markers (/lsa_markers):
     - 5 large sensor pads in front of the robot.
     - PURE GLOWING WHITE (1.0, 1.0, 1.0) when over white surface.
     - JET BLACK (0.05, 0.05, 0.05) when over black surface.
     - Floating text labels above each sensor showing [WHITE] or [BLACK] with %.
     - Published in 'base_link' frame for 100% reliable TF rendering.
  2. Terminal Live Readout:
     - Prints clear high-contrast ASCII bar graphs (BLACK vs WHITE) every 200ms.
  3. Uses qos_profile_sensor_data (BEST_EFFORT) to guarantee seamless micro-ROS connection.
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from visualization_msgs.msg import Marker, MarkerArray
from walle_interfaces.msg import LineSensorArray


class LsaVisualizerNode(Node):
    def __init__(self):
        super().__init__('lsa_visualizer_node')

        # Marker publisher for RViz
        self.marker_pub = self.create_publisher(MarkerArray, '/lsa_markers', 10)

        # Critical: qos_profile_sensor_data (BEST_EFFORT) matches micro-ROS publisher
        self.sub = self.create_subscription(
            LineSensorArray,
            '/line_sensors',
            self.lsa_callback,
            qos_profile_sensor_data
        )

        self.latest_msg = None
        self.msg_count = 0

        # Periodic timer for terminal high-contrast display (5 Hz)
        self.display_timer = self.create_timer(0.2, self.display_terminal)

        self.get_logger().info('LSA Independent Visualizer started.')
        self.get_logger().info('Subscribed to /line_sensors (BEST_EFFORT QoS).')
        self.get_logger().info('Publishing high-contrast markers to /lsa_markers.')

    def lsa_callback(self, msg: LineSensorArray):
        self.latest_msg = msg
        self.msg_count += 1

        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        # 5 sensors spaced along Y-axis in base_link coordinates:
        # Index 0 is Left (+Y), Index 4 is Right (-Y)
        sensor_y = [0.036, 0.018, 0.0, -0.018, -0.036]

        # ── 1. Base Mounting Plate (Dark Gray Frame under pads) ──────────────
        plate = Marker()
        plate.header.frame_id = 'base_link'
        plate.header.stamp = now
        plate.ns = 'lsa_base_plate'
        plate.id = 999
        plate.type = Marker.CUBE
        plate.action = Marker.ADD
        plate.pose.position.x = 0.065   # Just in front of robot body
        plate.pose.position.y = 0.0
        plate.pose.position.z = -0.026  # Just above ground
        plate.pose.orientation.w = 1.0
        plate.scale.x = 0.028
        plate.scale.y = 0.096
        plate.scale.z = 0.003
        plate.color.r = 0.15
        plate.color.g = 0.15
        plate.color.b = 0.15
        plate.color.a = 1.0
        marker_array.markers.append(plate)

        # ── 2. Individual Sensor Pads (High Contrast Black vs White) ─────────
        for i in range(5):
            norm_val = msg.normalized[i] if i < len(msg.normalized) else 0.0
            raw_val = msg.raw_adc[i] if i < len(msg.raw_adc) else 0.0
            is_white = norm_val > 0.60

            pad = Marker()
            pad.header.frame_id = 'base_link'
            pad.header.stamp = now
            pad.ns = 'lsa_pads'
            pad.id = i
            pad.type = Marker.CUBE
            pad.action = Marker.ADD

            pad.pose.position.x = 0.065
            pad.pose.position.y = sensor_y[i]
            pad.pose.orientation.w = 1.0

            pad.scale.x = 0.022
            pad.scale.y = 0.015

            if is_white:
                # WHITE LINE DETECTED: Bright pure white, pops up
                pad.pose.position.z = -0.018
                pad.scale.z = 0.012
                pad.color.r = 1.0
                pad.color.g = 1.0
                pad.color.b = 1.0
                pad.color.a = 1.0
            else:
                # BLACK FLOOR: Jet black tile, stays low
                pad.pose.position.z = -0.024
                pad.scale.z = 0.004
                pad.color.r = 0.04
                pad.color.g = 0.04
                pad.color.b = 0.04
                pad.color.a = 1.0

            marker_array.markers.append(pad)

            # ── Text Label above each sensor ──────────────────────────────────
            text_marker = Marker()
            text_marker.header.frame_id = 'base_link'
            text_marker.header.stamp = now
            text_marker.ns = 'lsa_labels'
            text_marker.id = 10 + i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = 0.065
            text_marker.pose.position.y = sensor_y[i]
            text_marker.pose.position.z = 0.008
            text_marker.scale.z = 0.012

            status_str = "WHITE" if is_white else "BLACK"
            text_marker.text = f"S{i}\n[{status_str}]\n{norm_val*100:.0f}%"

            if is_white:
                text_marker.color.r = 1.0
                text_marker.color.g = 1.0
                text_marker.color.b = 1.0
                text_marker.color.a = 1.0
            else:
                text_marker.color.r = 0.55
                text_marker.color.g = 0.55
                text_marker.color.b = 0.55
                text_marker.color.a = 0.85

            marker_array.markers.append(text_marker)

        # ── 3. Centroid Target Pointer ─────────────────────────────────────────
        pos_marker = Marker()
        pos_marker.header.frame_id = 'base_link'
        pos_marker.header.stamp = now
        pos_marker.ns = 'line_centroid'
        pos_marker.id = 100
        pos_marker.type = Marker.SPHERE
        pos_marker.action = Marker.ADD

        pos_marker.pose.position.x = 0.088
        pos_marker.pose.position.y = float(-msg.line_position * 0.008)
        pos_marker.pose.position.z = -0.018
        pos_marker.pose.orientation.w = 1.0
        pos_marker.scale.x = 0.020
        pos_marker.scale.y = 0.020
        pos_marker.scale.z = 0.020
        pos_marker.color.a = 1.0

        if msg.line_detected:
            # Neon Green when line is detected
            pos_marker.color.r = 0.0
            pos_marker.color.g = 1.0
            pos_marker.color.b = 0.1
        else:
            # Amber Yellow when lost
            pos_marker.color.r = 1.0
            pos_marker.color.g = 0.5
            pos_marker.color.b = 0.0

        marker_array.markers.append(pos_marker)
        self.marker_pub.publish(marker_array)

    def display_terminal(self):
        """Prints live high-contrast sensor states to terminal."""
        if self.latest_msg is None:
            return

        msg = self.latest_msg
        labels = ["S0 [L] ", "S1     ", "S2 [C] ", "S3     ", "S4 [R] "]
        lines = []

        lines.append("\n" + "=" * 65)
        lines.append("  LSA SENSOR LIVE CONTRAST (Independent Monitor)")
        lines.append("=" * 65)

        for i in range(5):
            norm = msg.normalized[i] if i < len(msg.normalized) else 0.0
            raw = int(msg.raw_adc[i]) if i < len(msg.raw_adc) else 0
            is_white = norm > 0.60
            state = "WHITE" if is_white else "BLACK"

            # 15-char visual bar
            filled = int(norm * 15)
            bar = "#" * filled + "-" * (15 - filled)

            lines.append(f"  {labels[i]} : [{bar}]  {norm*100:5.1f}%  |  Raw: {raw:4d}  |  {state}")

        det_str = "LOCKED" if msg.line_detected else "SEARCHING"
        lines.append("-" * 65)
        lines.append(f"  Line Status: {det_str} | Centroid: {msg.line_position:+.2f} | Packets: {self.msg_count}")
        lines.append("=" * 65)

        sys.stdout.write("\033[H\033[J" + "\n".join(lines) + "\n")
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = LsaVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
