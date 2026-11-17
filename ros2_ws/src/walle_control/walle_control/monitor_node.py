#!/usr/bin/env python3
"""
monitor_node.py
===============
A ROS2 node that subscribes to all telemetry topics published by the ESP32
and displays a live dashboard in the terminal.

WHAT THIS NODE DOES:
  - Subscribes to /balance_telemetry  → shows pitch, PID, motor duties
  - Subscribes to /line_sensors       → shows line position, sensor readings
  - Subscribes to /imu/data           → shows raw IMU data from MPU6050
  - Detects Wi-Fi/agent disconnection → warns you when data stops
  - Prints a formatted dashboard every second

WHY THIS NODE IS IMPORTANT:
  Without this node, your robot is a "black box" — you can't see what's
  happening inside. This is your window into the robot's mind.

  During PID tuning, you'll keep this terminal open alongside PlotJuggler
  to get quick numeric feedback while adjusting gains.

HOW TO RUN:
  ros2 run walle_control monitor_node

HOW IT FITS IN THE SYSTEM:
  ESP32 ──publishes──► /balance_telemetry ──► monitor_node (this file)
  ESP32 ──publishes──► /line_sensors      ──► monitor_node (this file)
"""

# ── Standard Python imports ───────────────────────────────────────────────────
import math         # For math.degrees() if we need radian conversion
import time         # For time.time() to detect message timeouts

# ── ROS2 Python library (rclpy) ───────────────────────────────────────────────
# rclpy is the official ROS2 Python client library.
# Think of it as the "SDK" that lets Python code talk to ROS2.
# It provides: Node class, Publisher, Subscriber, Timer, Service client, etc.
import rclpy
from rclpy.node import Node

# ── QoS (Quality of Service) settings ────────────────────────────────────────
# QoS controls HOW messages are delivered. Two main strategies:
#
# RELIABLE:   Every message is guaranteed to arrive. If lost, it's resent.
#             Like TCP in networking. Good for: service calls, important commands.
#             BAD for sensor data: if you miss a packet, you wait for the old one!
#
# BEST_EFFORT: Messages are sent once. If lost, it's dropped and you get the next.
#              Like UDP in networking. Good for: continuous sensor data.
#              If one pitch reading is lost, the NEXT reading (5ms later) is fine.
#
# For our telemetry topics (streaming sensor data), BEST_EFFORT is correct.
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ── Message types we subscribe to ────────────────────────────────────────────
# These are the classes rosidl generated from our .msg files (Phase 1)
from walle_interfaces.msg import BalanceTelemetry, LineSensorArray

# sensor_msgs/Imu is a standard ROS2 message type for IMU data.
# It contains: orientation quaternion, angular velocity, linear acceleration.
# The ESP32 will publish raw MPU6050 data in this standard format.
from sensor_msgs.msg import Imu


# ── QoS Profile for sensor data ───────────────────────────────────────────────
# We create a QoS profile that matches what the ESP32 publishes.
# micro-ROS typically uses BEST_EFFORT + KEEP_LAST(1) for sensor topics.
# If our profile doesn't match the publisher's profile, ROS2 won't connect them!
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,   # Drop lost packets, don't retry
    history=HistoryPolicy.KEEP_LAST,             # Only keep most recent messages
    depth=1                                       # Buffer size: 1 message
)


class MonitorNode(Node):
    """
    A ROS2 Node that subscribes to robot telemetry and shows a live dashboard.

    WHAT IS A NODE?
    In ROS2, a "Node" is a single process with a specific job.
    Think of nodes like workers in a factory:
    - One worker reads sensors (ESP32 node)
    - One worker displays data (this monitor node)
    - One worker records data (rosbag2 node)
    - They all talk to each other via topics

    A node is created by inheriting from rclpy.node.Node and calling
    super().__init__('node_name') to register it with the ROS2 system.
    """

    def __init__(self):
        # ── Initialize the parent Node class ──────────────────────────────────
        # 'walle_monitor' is the name this node advertises to the ROS2 system.
        # Run `ros2 node list` and you'll see /walle_monitor appear here.
        super().__init__('walle_monitor')

        # ── Internal state variables ───────────────────────────────────────────
        # We store the latest received message from each topic.
        # When the display timer fires, we read from these stored values.
        self.latest_telemetry = None   # Most recent BalanceTelemetry message
        self.latest_lsa = None         # Most recent LineSensorArray message
        self.latest_imu = None         # Most recent Imu message

        # Timestamps: when did we last receive each message?
        # Used to detect if the robot has gone offline (no messages for >1 second)
        self.last_telemetry_time = 0.0
        self.last_lsa_time = 0.0
        self.last_imu_time = 0.0

        # ── Create Subscribers ────────────────────────────────────────────────
        # A subscriber "listens" to a topic and calls a function every time
        # a new message arrives. The function is called a "callback".
        #
        # Syntax: self.create_subscription(MessageType, 'topic_name', callback, qos)
        #
        # When a BalanceTelemetry message arrives on /balance_telemetry,
        # ROS2 automatically calls self.telemetry_callback(msg).
        self.telemetry_sub = self.create_subscription(
            BalanceTelemetry,              # Message type to expect
            '/balance_telemetry',          # Topic name to listen to
            self.telemetry_callback,       # Function to call when message arrives
            SENSOR_QOS                     # Quality of service settings
        )

        self.lsa_sub = self.create_subscription(
            LineSensorArray,
            '/line_sensors',
            self.lsa_callback,
            SENSOR_QOS
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            SENSOR_QOS
        )

        # ── Create a Timer for the display ────────────────────────────────────
        # A ROS2 timer calls a function at a fixed interval.
        # We want to refresh the dashboard every 0.5 seconds (2 Hz).
        # The sensor callbacks run much faster (50 Hz) but we only need to
        # display at a human-readable rate.
        #
        # 0.5 = period in seconds → fires every 500ms
        self.display_timer = self.create_timer(0.5, self.display_callback)

        # Log a startup message (appears in terminal when node starts)
        self.get_logger().info('Wall-E Monitor Node started. Waiting for robot...')

    # ── Subscriber Callbacks ──────────────────────────────────────────────────
    # These functions are called by ROS2 automatically whenever a new
    # message arrives on the subscribed topic. They run in ROS2's executor thread.

    def telemetry_callback(self, msg: BalanceTelemetry):
        """Called every time a new BalanceTelemetry message arrives (~50 Hz)."""
        self.latest_telemetry = msg
        self.last_telemetry_time = time.time()  # Record arrival time

    def lsa_callback(self, msg: LineSensorArray):
        """Called every time a new LineSensorArray message arrives (~50 Hz)."""
        self.latest_lsa = msg
        self.last_lsa_time = time.time()

    def imu_callback(self, msg: Imu):
        """Called every time a new Imu message arrives (~100 Hz)."""
        self.latest_imu = msg
        self.last_imu_time = time.time()

    # ── Display Callback ──────────────────────────────────────────────────────
    def display_callback(self):
        """
        Called every 0.5 seconds by the display timer.
        Reads the latest stored messages and prints a formatted dashboard.
        """
        now = time.time()

        # Clear the terminal screen for a live-updating display effect
        # '\033[2J' = ANSI escape code to clear screen
        # '\033[H'  = ANSI escape code to move cursor to top-left
        print('\033[2J\033[H', end='')

        print('=' * 60)
        print('      Wall-E ROS2 Robot - Live Monitor')
        print('=' * 60)

        # ── Connection Status ─────────────────────────────────────────────────
        # Check if we're receiving data. If last message was > 2 seconds ago,
        # the robot is likely offline or Wi-Fi dropped.
        telemetry_age = now - self.last_telemetry_time

        if self.latest_telemetry is None:
            print('\n  [WAITING] Waiting for robot to connect...')
            print('     Make sure:')
            print('     1. micro-ROS agent is running (ros2 run micro_ros_agent ...)')
            print('     2. ESP32 is powered on and connected to Wi-Fi')
            print('     3. Both are on the same network\n')
            return

        if telemetry_age > 2.0:
            print(f'\n  [WARNING] No data for {telemetry_age:.1f}s - Robot offline?')
            print(f'     Last seen: {telemetry_age:.1f} seconds ago\n')

        tel = self.latest_telemetry  # Shorthand alias

        # ── Mode Display ──────────────────────────────────────────────────────
        mode_names = {0: 'IDLE', 1: 'BALANCE_ONLY', 2: 'LINE_FOLLOW', 3: 'BOTH'}
        mode_str = mode_names.get(tel.mode, f'UNKNOWN({tel.mode})')

        print(f'\n  MODE: {mode_str}')
        wifi_str = 'CONNECTED' if tel.wifi_connected else 'DISCONNECTED'
        agent_str = 'ACTIVE' if tel.agent_connected else 'NOT FOUND'
        print(f'  Wi-Fi: {wifi_str}  |  Agent: {agent_str}')

        # ── Balance Data ──────────────────────────────────────────────────────
        print('\n  -- Balance Controller ----------------------------------')

        # Visual pitch bar: shows tilt direction graphically
        # A bar that goes left when leaning back, right when leaning forward
        pitch = tel.pitch_angle
        bar_width = 20
        bar_pos = int((pitch / 40.0) * bar_width) + bar_width  # Map -40..+40 to 0..40
        bar_pos = max(0, min(bar_width * 2, bar_pos))
        bar = ['-'] * (bar_width * 2 + 1)
        bar[bar_width] = '|'  # Center marker
        bar[bar_pos] = '#'    # Robot position
        bar_str = ''.join(bar)

        pitch_warn = ' [DANGER!]' if abs(pitch) > 35.0 else ''
        print(f'  Pitch angle:   {pitch:+7.2f}°  [{bar_str}]{pitch_warn}')
        print(f'  Pitch rate:    {tel.pitch_rate:+7.2f} °/s')
        print(f'  Target pitch:  {tel.target_pitch:+7.2f}°')
        print(f'  PID output:    {tel.pid_output:+7.1f}%')

        # PID terms bar for visual feedback
        print(f'\n  PID terms:')
        print(f'    P = {tel.pid_p_term:+8.2f}  (Kp x error)')
        print(f'    I = {tel.pid_i_term:+8.2f}  (Ki x accumulated error)')
        print(f'    D = {tel.pid_d_term:+8.2f}  (Kd x rate of change)')

        # ── Motor Data ────────────────────────────────────────────────────────
        print('\n  -- Motors ----------------------------------------------')

        def motor_bar(duty):
            """Create a simple visual bar for motor duty cycle."""
            filled = int(abs(duty) / 100.0 * 10)
            if duty >= 0:
                return f'[{"#" * filled:10s}] +{duty:.1f}%'
            else:
                return f'[{"#" * filled:10s}] {duty:.1f}%'

        print(f'  Left  motor:  {motor_bar(tel.motor_left_duty)}')
        print(f'  Right motor:  {motor_bar(tel.motor_right_duty)}')
        print(f'  Steering offset: {tel.steering_offset:+.2f}')

        # ── Line Sensor Data ──────────────────────────────────────────────────
        if self.latest_lsa is not None:
            lsa = self.latest_lsa
            print('\n  -- Line Sensors (IR Array) -----------------------------')
            detected_str = 'LINE DETECTED' if lsa.line_detected else 'LINE LOST'
            print(f'  Status: {detected_str}')
            print(f'  Position: {lsa.line_position:+.3f}  (-1=left, 0=center, +1=right)')

            # Visual position bar
            pos = lsa.line_position
            bar_p = int((pos + 1.0) / 2.0 * 20)  # Map -1..+1 to 0..20
            bar_p = max(0, min(20, bar_p))
            pos_bar = ['.'] * 21
            pos_bar[10] = '|'   # Center
            pos_bar[bar_p] = '#'
            print(f'  [{" ".join(pos_bar)}]')
            print(f'   L                  C                  R')

            # Normalized sensor readings
            norm = lsa.normalized
            bars = ''.join(['█' if n > 0.7 else '▒' if n > 0.3 else '░' for n in norm])
            print(f'  Sensors: [{bars}]  S0:{norm[0]:.2f} S1:{norm[1]:.2f} '
                  f'S2:{norm[2]:.2f} S3:{norm[3]:.2f} S4:{norm[4]:.2f}')

        # ── IMU Raw Data ──────────────────────────────────────────────────────
        if self.latest_imu is not None:
            imu = self.latest_imu
            print('\n  ── IMU Raw (MPU6050) ───────────────────────────────────')
            ax = imu.linear_acceleration.x
            ay = imu.linear_acceleration.y
            az = imu.linear_acceleration.z
            gx = imu.angular_velocity.x
            print(f'  Accel: x={ax:+.2f} y={ay:+.2f} z={az:+.2f} m/s²')
            print(f'  Gyro:  x={gx:+.2f} rad/s')

        print('\n' + '=' * 60)
        print('  Press Ctrl+C to stop  |  Use tuner_node for PID tuning')
        print('=' * 60)


def main(args=None):
    """
    Entry point for the monitor node.

    HOW ROS2 NODES START:
    1. rclpy.init()         → Initialize the ROS2 Python library
    2. Create the node      → Registers with ROS2 middleware
    3. rclpy.spin(node)     → Hand control to ROS2's event loop
                               (this blocks and keeps running,
                                calling callbacks as messages arrive)
    4. rclpy.shutdown()     → Clean up when Ctrl+C is pressed
    """
    rclpy.init(args=args)           # Step 1: Boot up ROS2 Python layer
    node = MonitorNode()            # Step 2: Create our node (registers subscriptions)
    try:
        rclpy.spin(node)            # Step 3: Run forever, calling callbacks as msgs arrive
    except KeyboardInterrupt:
        pass                        # Ctrl+C pressed — exit gracefully
    finally:
        node.destroy_node()         # Clean up the node's resources
        rclpy.shutdown()            # Step 4: Shut down ROS2 Python layer


if __name__ == '__main__':
    main()
