import time

from geometry_msgs.msg import Twist
from rclpy.node import Node


class RobotAPI:
    """Restricted robot control API exposed to generated policy code."""

    def __init__(self, node: Node):
        self.node = node
        self._cmd_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)

    def say(self, text: str) -> None:
        self.node.get_logger().info(f"[ROBOT SAY] {text}")

    def navigate_to(self, location: str) -> None:
        # TODO: replace with Nav2 action call.
        self.node.get_logger().info(f"[NAVIGATE] Going to: {location}")

    def stop(self) -> None:
        msg = Twist()
        self._cmd_pub.publish(msg)
        self.node.get_logger().info('[STOP] Robot stopping')

    def save_waypoint(self, name: str) -> None:
        # TODO: replace with waypoint persistence logic.
        self.node.get_logger().info(f"[WAYPOINT] Saving current pose as: {name}")

    def drive(self, linear_x: float, angular_z: float, duration_s: float) -> None:
        duration = max(0.0, min(float(duration_s), 10.0))
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)

        self.node.get_logger().info(
            f"[DRIVE] linear_x={cmd.linear.x}, angular_z={cmd.angular.z}, duration_s={duration}"
        )

        end_at = time.monotonic() + duration
        while time.monotonic() < end_at:
            self._cmd_pub.publish(cmd)
            time.sleep(0.1)

        self.stop()

    def come_back(self) -> None:
        # TODO: replace with actual return-to-home behavior.
        self.node.get_logger().info('[RETURN] Returning to base')
