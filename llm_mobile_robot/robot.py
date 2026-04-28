import time
from dotenv import load_dotenv
from elevenlabs.client import ElevenLabs
from elevenlabs.play import play
import os
import math

from geometry_msgs.msg import Twist, PoseWithCovarianceStamped,Quaternion,PoseStamped
from rclpy.node import Node



load_dotenv()
elevenlabs = ElevenLabs(
  api_key=os.getenv("ELEVENLABS_API_KEY"),
)

def quat_to_yaw_deg(q: Quaternion) -> float:
    # Planar yaw only
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


class RobotAPI:

    def __init__(self, node: Node):
        self.node = node
        self.current_pose = None

        self._cmd_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_pose, 10)
    
    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose
        pose = {
            "x": float(p.position.x),
            "y": float(p.position.y),
            "yaw_deg": float(quat_to_yaw_deg(p.orientation))
        }
        self.current_pose = pose

    def say(self, text: str) -> None:
        audio = elevenlabs.text_to_speech.convert(
            text=text,
            voice_id="hpp4J3VqNfWAUOO0d1Us",  # "George" - browse voices at elevenlabs.io/app/voice-library
            model_id="eleven_v3",
            output_format="mp3_44100_128",
)
        play(audio)
        self.node.get_logger().info(f"[ROBOT SAY] {text}")

    def navigate_to(self, location: str) -> None:
        # TODO: replace with Nav2 action call.
        self.node.get_logger().info(f"[NAVIGATE] Going to: {location}")

    def stop(self) -> None:
        msg = Twist()
        self._cmd_pub.publish(msg)
        self.node.get_logger().info('[STOP] Robot stopping')

    def save_waypoint(self, name: str) -> None:
        if self.current_pose is None:
            self.node.get_logger().warn(f"Cannot save waypoint '{name}': current pose unknown")
            return
        self.waypoints[name] = self.current_pose

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
