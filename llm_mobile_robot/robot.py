import math
import os
import time

from dotenv import load_dotenv
from elevenlabs.client import ElevenLabs
from elevenlabs.play import play

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

from llm_mobile_robot.waypoint_store import WaypointStore


load_dotenv()
elevenlabs = ElevenLabs(
    api_key=os.getenv('ELEVENLABS_API_KEY'),
)


def quat_to_yaw_deg(q: Quaternion) -> float:
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def yaw_deg_to_quat(yaw_deg: float) -> Quaternion:
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
        self.current_pose: dict[str, float] | None = None

        waypoint_file = os.environ.get('WAYPOINTS_FILE', '/home/brume/Documents/maps/sim_world/waypoints.json')
        self._waypoint_store = WaypointStore(waypoint_file)
        self.waypoints = self._waypoint_store.load()

        self._cmd_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_pose, 10)

        self._nav_action = ActionClient(self.node, NavigateToPose, '/navigate_to_pose')

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose
        pose = {
            'x': float(p.position.x),
            'y': float(p.position.y),
            'yaw_deg': float(quat_to_yaw_deg(p.orientation)),
        }
        self.current_pose = pose

    def status_snapshot(self) -> dict[str, float | bool | str]:
        return {
            'battery_percent': float(os.environ.get('SIM_BATTERY_PERCENT', '100')),
            'busy': os.environ.get('SIM_IS_BUSY', 'false').strip().lower() == 'true',
            'obstacle_detected': os.environ.get('SIM_OBSTACLE_DETECTED', 'false').strip().lower() == 'true',
            'current_location_known': self.current_pose is not None,
        }

    def say(self, text: str) -> None:
        audio = elevenlabs.text_to_speech.convert(
            text=text,
            voice_id='hpp4J3VqNfWAUOO0d1Us',
            model_id='eleven_v3',
            output_format='mp3_44100_128',
        )
        play(audio)
        self.node.get_logger().info(f'[ROBOT SAY] {text}')

    def navigate_to(self, location: str) -> None:
        waypoint = self.waypoints.get(location)
        if waypoint is None:
            self.node.get_logger().warn(f"[NAVIGATE] Unknown waypoint '{location}'")
            self.say(f"I do not know waypoint {location}.")
            self.stop()
            return

        if not self._nav_action.wait_for_server(timeout_sec=3.0):
            self.node.get_logger().error('[NAVIGATE] /navigate_to_pose action server not available')
            self.say('Navigation server is not available right now.')
            self.stop()
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(waypoint['x'])
        goal.pose.pose.position.y = float(waypoint['y'])
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation = yaw_deg_to_quat(float(waypoint.get('yaw_deg', 0.0)))

        self.node.get_logger().info(
            f"[NAVIGATE] Sending goal '{location}' x={goal.pose.pose.position.x:.2f}, y={goal.pose.pose.position.y:.2f}"
        )

        send_goal_future = self._nav_action.send_goal_async(goal)
        while not send_goal_future.done():
            time.sleep(0.05)

        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error(f"[NAVIGATE] Goal rejected for '{location}'")
            self.say(f'I could not navigate to {location}.')
            self.stop()
            return

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            time.sleep(0.1)

        result = result_future.result()
        if result is None:
            self.node.get_logger().error(f"[NAVIGATE] No result for '{location}'")
            self.say(f'Navigation to {location} failed.')
            self.stop()
            return

        self.node.get_logger().info(f"[NAVIGATE] Goal completed for '{location}'")

    def stop(self) -> None:
        msg = Twist()
        self._cmd_pub.publish(msg)
        self.node.get_logger().info('[STOP] Robot stopping')

    def save_waypoint(self, name: str) -> None:
        if self.current_pose is None:
            self.node.get_logger().warn(f"Cannot save waypoint '{name}': current pose unknown")
            return

        self._waypoint_store.set_waypoint(name, self.current_pose)
        self._waypoint_store.save()
        self.waypoints = self._waypoint_store.waypoints
        self.node.get_logger().info(f"[WAYPOINT] Saved '{name}' to {self._waypoint_store.file_path}")

    def drive(self, linear_x: float, angular_z: float, duration_s: float) -> None:
        duration = max(0.0, min(float(duration_s), 10.0))
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)

        self.node.get_logger().info(
            f'[DRIVE] linear_x={cmd.linear.x}, angular_z={cmd.angular.z}, duration_s={duration}'
        )

        end_at = time.monotonic() + duration
        while time.monotonic() < end_at:
            self._cmd_pub.publish(cmd)
            time.sleep(0.1)

        self.stop()

    def come_back(self) -> None:
        if 'entrance' in self.waypoints:
            self.navigate_to('entrance')
            return
        if 'base' in self.waypoints:
            self.navigate_to('base')
            return
        self.node.get_logger().warn('[RETURN] No entrance/base waypoint configured')
        self.stop()
