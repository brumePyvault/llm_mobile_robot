import time
from dotenv import load_dotenv
from elevenlabs.client import ElevenLabs
from elevenlabs.play import play
import os
import math
import subprocess
import yaml

import rclpy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Quaternion, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

from llm_mobile_robot.waypoint_store import WaypointStore


load_dotenv()

elevenlabs = ElevenLabs(
    api_key=os.getenv("ELEVENLABS_API_KEY"),
)


def quat_to_yaw_deg(q: Quaternion) -> float:
    """
    Convert quaternion orientation to planar yaw angle in degrees.
    """
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def yaw_deg_to_quat(yaw_deg: float) -> Quaternion:
    """
    Convert yaw angle in degrees to quaternion.
    """
    yaw_rad = math.radians(yaw_deg)

    quat = Quaternion()
    quat.x = 0.0
    quat.y = 0.0
    quat.z = math.sin(yaw_rad / 2)
    quat.w = math.cos(yaw_rad / 2)

    return quat


class RobotAPI:

    def __init__(self, node: Node):
        self.node = node

        self.current_pose = None
        self.current_pose_time = None

        self.prev_pose = None
        self.current_goal = None

        waypoint_file = os.environ.get(
            "WAYPOINTS_FILE",
            "/home/brume/Documents/maps/sim_world/waypoints.json"
        )

        self._waypoint_store = WaypointStore(waypoint_file)
        self.waypoints = self._waypoint_store.load()

        self._cmd_pub = self.node.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        self._pose_sub = self.node.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._on_pose,
            10
        )

        self._nav_action = ActionClient(
            self.node,
            NavigateToPose,
            "navigate_to_pose"
        )

        # On startup, block briefly for an initial AMCL pose so the robot has
        # a known map-frame pose before first command execution.
        initial_pose = self.get_amcl_pose_from_terminal(timeout_s=60.0)
        if initial_pose is not None:
            self.current_pose = initial_pose
            self.current_pose_time = time.monotonic()
            self.node.get_logger().info(
                "[STARTUP] Initial AMCL pose acquired from terminal."
            )
        else:
            self.node.get_logger().warn(
                "[STARTUP] Initial AMCL pose not available yet; continuing without it."
            )

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        """
        Subscriber callback for /amcl_pose.
        This is the main and preferred method for getting the robot pose.
        """
        p = msg.pose.pose

        pose = {
            "x": float(p.position.x),
            "y": float(p.position.y),
            "yaw_deg": float(quat_to_yaw_deg(p.orientation))
        }

        self.current_pose = pose
        self.current_pose_time = time.monotonic()

        self.node.get_logger().debug(
            f"[POSE] x={pose['x']:.2f}, "
            f"y={pose['y']:.2f}, "
            f"yaw_deg={pose['yaw_deg']:.1f}"
        )

    def get_amcl_pose_from_terminal(self, timeout_s: float = 5.0):
        """
        Fallback method.

        This directly runs:

            ros2 topic echo --once /amcl_pose

        Then it extracts x, y, and yaw_deg from the terminal output.

        This should not replace the subscriber. It is only a backup in case
        self.current_pose is None or stale.
        """
        try:
            result = subprocess.run(
                ["ros2", "topic", "echo", "--once", "/amcl_pose"],
                capture_output=True,
                text=True,
                timeout=timeout_s
            )

            if result.returncode != 0:
                self.node.get_logger().warn(
                    f"[AMCL CLI] Failed to read /amcl_pose: {result.stderr}"
                )
                return None

            output = result.stdout.strip()

            if not output:
                self.node.get_logger().warn("[AMCL CLI] Empty /amcl_pose output")
                return None

            output = output.replace("---", "").strip()

            data = yaml.safe_load(output)

            p = data["pose"]["pose"]

            x = float(p["position"]["x"])
            y = float(p["position"]["y"])

            q = Quaternion()
            q.x = float(p["orientation"]["x"])
            q.y = float(p["orientation"]["y"])
            q.z = float(p["orientation"]["z"])
            q.w = float(p["orientation"]["w"])

            yaw_deg = quat_to_yaw_deg(q)

            pose = {
                "x": x,
                "y": y,
                "yaw_deg": yaw_deg
            }

            self.node.get_logger().info(
                f"[AMCL CLI] x={pose['x']:.2f}, "
                f"y={pose['y']:.2f}, "
                f"yaw_deg={pose['yaw_deg']:.1f}"
            )

            return pose

        except subprocess.TimeoutExpired:
            self.node.get_logger().warn(
                "[AMCL CLI] Timeout waiting for /amcl_pose"
            )
            return None

        except Exception as e:
            self.node.get_logger().error(
                f"[AMCL CLI] Error reading AMCL pose: {e}"
            )
            return None

    def get_current_pose(self, max_age_s: float = 2.0):

        terminal_pose = self.get_amcl_pose_from_terminal(timeout_s=3.0)

        if terminal_pose is not None:
            self.current_pose = terminal_pose
            self.current_pose_time = time.monotonic()
            return terminal_pose

        return self.current_pose

    def say(self, text: str) -> None:
        audio = elevenlabs.text_to_speech.convert(
            text=text,
            voice_id="hpp4J3VqNfWAUOO0d1Us",
            model_id="eleven_v3",
            output_format="mp3_44100_128",
        )

        play(audio)

        self.node.get_logger().info(f"[ROBOT SAY] {text}")

    def navigate_to(self, location: str) -> None:
        """
        Navigate to a saved waypoint.
        """
        self.prev_pose = self.get_current_pose()

        waypoint = self.waypoints.get(location)

        if waypoint is None:
            self.node.get_logger().warn(
                f"[NAVIGATE] Unknown waypoint '{location}'"
            )
            self.say(f"I do not know waypoint {location}.")
            self.stop()
            return

        if not self._nav_action.wait_for_server(timeout_sec=3.0):
            self.node.get_logger().error(
                "[NAVIGATE] /navigate_to_pose action server not available"
            )
            self.say("Navigation server is not available right now.")
            self.stop()
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()

        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()

        goal.pose.pose.position.x = float(waypoint["x"])
        goal.pose.pose.position.y = float(waypoint["y"])
        goal.pose.pose.position.z = 0.0

        goal.pose.pose.orientation = yaw_deg_to_quat(
            float(waypoint.get("yaw_deg", 0.0))
        )

        self.current_goal = {
            "x": goal.pose.pose.position.x,
            "y": goal.pose.pose.position.y,
        }

        self.node.get_logger().info(
            f"[NAVIGATE] Sending goal '{location}' "
            f"x={goal.pose.pose.position.x:.2f}, "
            f"y={goal.pose.pose.position.y:.2f}"
        )

        future = self._nav_action.send_goal_async(
            goal,
            feedback_callback=self._on_nav_feedback
        )

        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().warn("[NAVIGATE] Goal was rejected")
            self.say("Goal was rejected.")
            return

        self.node.get_logger().info("[NAVIGATE] Goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)

    def _on_nav_feedback(self, feedback_msg):
        """
        Optional feedback callback from Nav2.
        You can later use this to log distance remaining if needed.
        """
        pass

    def _on_nav_result(self, future):
        """
        Called when Nav2 reports the navigation action result.
        """
        result = future.result()

        if result is None:
            self.node.get_logger().warn("[NAVIGATE] No result returned")
            return

        self.node.get_logger().info("[NAVIGATE] Navigation action completed")

    def wait_until_navigate_done(
        self,
        tolerance: float = 0.20,
        timeout_s: float = 60.0
    ) -> bool:
        """
        Wait until the robot reaches the current navigation goal.

        This function checks the robot's current AMCL pose against the
        target goal position.

        It uses:
        1. The /amcl_pose subscriber first.
        2. The terminal fallback if the subscriber pose is missing or stale.
        """
        if self.current_goal is None:
            self.node.get_logger().warn("[NAVIGATE] No active goal")
            return False

        start_time = time.monotonic()

        while True:
            # Allow ROS callbacks to run.
            rclpy.spin_once(self.node, timeout_sec=0.1)

            pose = self.get_current_pose(max_age_s=2.0)

            if pose is None:
                self.node.get_logger().warn(
                    "[NAVIGATE] Waiting for current pose..."
                )

                if time.monotonic() - start_time > timeout_s:
                    self.node.get_logger().warn(
                        "[NAVIGATE] Timeout: no AMCL pose available"
                    )
                    return False

                continue

            # Recalculate distance every loop.
            dx = self.current_goal["x"] - pose["x"]
            dy = self.current_goal["y"] - pose["y"]
            distance = math.sqrt(dx * dx + dy * dy)

            if distance <= tolerance:
                self.node.get_logger().info(
                    "[NAVIGATE] Goal reached within tolerance"
                )
                self.current_goal = None
                return True

            if time.monotonic() - start_time > timeout_s:
                self.node.get_logger().warn(
                    "[NAVIGATE] Timeout while waiting for goal"
                )
                return False

    def stop(self) -> None:
        """
        Stop robot movement by publishing zero velocity.
        """
        msg = Twist()
        self._cmd_pub.publish(msg)

        self.node.get_logger().info("[STOP] Robot stopping")

    def save_waypoint(self, name: str) -> None:
        """
        Save the robot's current AMCL pose as a named waypoint.
        """
        pose = self.get_current_pose()

        if pose is None:
            self.node.get_logger().warn(
                f"[WAYPOINT] Cannot save waypoint '{name}': current pose unknown"
            )
            return

        self._waypoint_store.set_waypoint(name, pose)
        self._waypoint_store.save()

        self.waypoints = self._waypoint_store.waypoints

        self.node.get_logger().info(
            f"[WAYPOINT] Saved '{name}' to {self._waypoint_store.file_path}"
        )

    def drive(
        self,
        linear_x: float,
        angular_z: float,
        duration_s: float
    ) -> None:
        """
        Drive the robot manually using /cmd_vel for a fixed duration.
        """
        self.prev_pose = self.get_current_pose()

        duration = max(0.0, min(float(duration_s), 10.0))

        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)

        self.node.get_logger().info(
            f"[DRIVE] linear_x={cmd.linear.x}, "
            f"angular_z={cmd.angular.z}, "
            f"duration_s={duration}"
        )

        end_at = time.monotonic() + duration

        while time.monotonic() < end_at:
            self._cmd_pub.publish(cmd)

            # Better than only time.sleep, because this still allows ROS callbacks.
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.stop()

    def come_back(self) -> None:
        """
        Return to the robot's previous pose.
        """
        if self.prev_pose is None:
            self.node.get_logger().warn("[COME BACK] Previous pose is unknown")
            self.stop()
            return

        if not self._nav_action.wait_for_server(timeout_sec=3.0):
            self.node.get_logger().error(
                "[COME BACK] /navigate_to_pose action server not available"
            )
            self.stop()
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()

        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()

        goal.pose.pose.position.x = float(self.prev_pose["x"])
        goal.pose.pose.position.y = float(self.prev_pose["y"])
        goal.pose.pose.position.z = 0.0

        goal.pose.pose.orientation = yaw_deg_to_quat(
            float(self.prev_pose.get("yaw_deg", 0.0))
        )

        self.current_goal = {
            "x": goal.pose.pose.position.x,
            "y": goal.pose.pose.position.y,
        }

        self.node.get_logger().info(
            f"[COME BACK] Returning to previous location "
            f"x={goal.pose.pose.position.x:.2f}, "
            f"y={goal.pose.pose.position.y:.2f}"
        )

        send_goal_future = self._nav_action.send_goal_async(goal,feedback_callback=self._on_nav_feedback)
        send_goal_future.add_done_callback(self._on_goal_response)

        

        