import rclpy
import os
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.node import Node
from dotenv import load_dotenv

load_dotenv()

class LLMControlNode(Node):

    def __init__(self) -> None:
        super().__init__('llm_control')

        self.voice_topic = os.environ.get("VOICE_TOPIC", "/voice/text")

        self.create_subscription(
            String,
            self.voice_topic,
            self._on_voice_text,
            10
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def _on_voice_text(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return

        self.get_logger().info(f"Voice text: {text}")

        