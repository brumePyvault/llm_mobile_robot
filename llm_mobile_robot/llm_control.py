import ast
import os
import textwrap
from typing import Any

import rclpy
from dotenv import load_dotenv
from openai import OpenAI
from rclpy.node import Node
from std_msgs.msg import String
import math
import time

from llm_mobile_robot.robot import RobotAPI

load_dotenv()


DEFAULT_POLICY_PROMPT = textwrap.dedent(
    """
    You translate user voice commands into executable Python policy code.

    Rules:
    1) Return ONLY Python code. No markdown fences or explanations.
    2) Define a function named `run(robot)`.
    3) Use only these robot API methods:
       - robot.say(text: str)
       - robot.navigate_to(location: str)
       - robot.stop()
       - robot.wait_until_navigate_done()
       - robot.save_waypoint(name: str)
       - robot.drive(linear_x: float, angular_z: float, duration_s: float)
       - robot.come_back()
    4) Keep actions concise and safe.
    """
).strip()

FEW_SHOT_EXAMPLES = textwrap.dedent(
    """
    Example command: "Go to the office."
    World context:
    Current robot pose: x=0.00, y=0.00, yaw_deg=0.0
    Known locations and distances from robot:
    - entrance: 0.00 metres away, at (x=0.00, y=0.00)
    - office: 2.69 metres away, at (x=2.69, y=0.00)
    - kitchen: 6.08 metres away, at (x=6.08, y=0.00)
    - meeting_room: 5.15 metres away, at (x=5.15, y=0.00)
    - printer: 4.03 metres away, at (x=4.03, y=0.00)
    - charging_station: 1.80 metres away, at (x=1.80, y=0.00)

    Example output:
    def run(robot):
        robot.say("Going to the office.")
        robot.navigate_to("office")


    Example command: "Go to the closest location between the kitchen and office."
    World context:
    Current robot pose: x=0.00, y=0.00, yaw_deg=0.0
    Known locations and distances from robot:
    - office: 2.69 metres away, at (x=2.69, y=0.00)
    - kitchen: 6.08 metres away, at (x=6.08, y=0.00)

    Example output:
    def run(robot):
        robot.say("The office is closer than the kitchen. Going to the office.")
        robot.navigate_to("office")


    Example command: "Visit the printer, then the meeting room, then stop."
    World context:
    Current robot pose: x=0.00, y=0.00, yaw_deg=0.0
    Known locations and distances from robot:
    - printer: 4.03 metres away, at (x=4.03, y=0.00)
    - meeting_room: 5.15 metres away, at (x=5.15, y=0.00)

    Example output:
    def run(robot):
        robot.say("I will visit the printer, then the meeting room.")
        robot.navigate_to("printer")
        robot.wait_until_navigate_done()
        robot.navigate_to("meeting_room")
        robot.wait_until_navigate_done()
        robot.stop()


    Example command: "Visit the printer, kitchen, and meeting room in the shortest estimated order, starting from my current position."
    World context:
    Current robot pose: x=1.33, y=1.67, yaw_deg=-22.9
    Known locations and distances from robot:
    - kitchen: 2.43 metres away, at (x=-1.00, y=2.35)
    - meeting_room: 2.49 metres away, at (x=-0.44, y=-0.08)
    - printer: 3.58 metres away, at (x=-0.40, y=-1.47)

    Route reasoning to follow silently:
    Compare all valid orders using coordinates and total route distance.
    Do not only rank destinations by distance from the starting position.
    After each movement, update the reference position to the last visited waypoint.
    The shortest estimated route is kitchen, meeting_room, printer.

    Example output:
    def run(robot):
        robot.say("I will visit the kitchen, then the meeting room, then the printer.")
        robot.navigate_to("kitchen")
        robot.wait_until_navigate_done()
        robot.navigate_to("meeting_room")
        robot.wait_until_navigate_done()
        robot.navigate_to("printer")
        robot.wait_until_navigate_done()


    Example command: "Visit the office, printer, and kitchen in the shortest estimated order."
    World context:
    Current robot pose: x=1.28, y=-0.19, yaw_deg=4.4
    Known locations and distances from robot:
    - kitchen: 3.42 metres away, at (x=-1.00, y=2.35)
    - office: 2.53 metres away, at (x=-0.94, y=1.02)
    - printer: 2.10 metres away, at (x=-0.40, y=-1.47)

    Route reasoning to follow silently:
    Compare all valid route orders using the current pose and waypoint coordinates.
    The shortest estimated route is printer, office, kitchen.

    Example output:
    def run(robot):
        robot.say("I will visit the printer, then the office, then the kitchen.")
        robot.navigate_to("printer")
        robot.wait_until_navigate_done()
        robot.navigate_to("office")
        robot.wait_until_navigate_done()
        robot.navigate_to("kitchen")
        robot.wait_until_navigate_done()


    Example command: "Go to the farthest location first, then visit the closest remaining location."
    World context:
    Current robot pose: x=-0.46, y=-1.46, yaw_deg=-10.3
    Known locations and distances from robot:
    - charging_station: 2.78 metres away, at (x=1.56, y=0.45)
    - entrance: 2.91 metres away, at (x=-2.63, y=0.49)
    - home: 1.74 metres away, at (x=-2.12, y=-0.95)
    - kitchen: 3.85 metres away, at (x=-1.00, y=2.35)
    - meeting_room: 1.38 metres away, at (x=-0.44, y=-0.08)
    - office: 2.52 metres away, at (x=-0.94, y=1.02)
    - printer: 0.07 metres away, at (x=-0.40, y=-1.47)

    Route reasoning to follow silently:
    First choose the farthest location from the current robot pose.
    The farthest location is kitchen.
    Then update the reference position to kitchen.
    From kitchen, the closest remaining location is office, not printer.

    Example output:
    def run(robot):
        robot.say("I will go to the kitchen first, then the office.")
        robot.navigate_to("kitchen")
        robot.wait_until_navigate_done()
        robot.navigate_to("office")
        robot.wait_until_navigate_done()


    Example command: "Visit the kitchen after the office, but decide where the printer should fit in the route."
    World context:
    Current robot pose: x=1.39, y=0.81, yaw_deg=80.7
    Known locations and distances from robot:
    - kitchen: 2.84 metres away, at (x=-1.00, y=2.35)
    - office: 2.34 metres away, at (x=-0.94, y=1.02)
    - printer: 2.90 metres away, at (x=-0.40, y=-1.47)

    Route reasoning to follow silently:
    The kitchen must come after the office.
    Compare all valid orders that satisfy this constraint.
    Valid orders include printer-office-kitchen, office-printer-kitchen, and office-kitchen-printer.
    The shortest valid order is printer, office, kitchen.

    Example output:
    def run(robot):
        robot.say("I will visit the printer, then the office, then the kitchen.")
        robot.navigate_to("printer")
        robot.wait_until_navigate_done()
        robot.navigate_to("office")
        robot.wait_until_navigate_done()
        robot.navigate_to("kitchen")
        robot.wait_until_navigate_done()


    Example command: "Go to the office last after visiting the kitchen and printer efficiently."
    World context:
    Current robot pose: x=-1.02, y=2.38, yaw_deg=0.0
    Known locations and distances from robot:
    - kitchen: 0.04 metres away, at (x=-1.00, y=2.35)
    - office: 1.37 metres away, at (x=-0.94, y=1.02)
    - printer: 3.90 metres away, at (x=-0.40, y=-1.47)

    Route reasoning to follow silently:
    The office must be the final destination.
    Compare only valid orders that end at office.
    The efficient valid route is kitchen, printer, office.

    Example output:
    def run(robot):
        robot.say("I will visit the kitchen, then the printer, and finish at the office.")
        robot.navigate_to("kitchen")
        robot.wait_until_navigate_done()
        robot.navigate_to("printer")
        robot.wait_until_navigate_done()
        robot.navigate_to("office")
        robot.wait_until_navigate_done()


    Example command: "Visit the printer first, then choose the shortest route between the kitchen and meeting room."
    World context:
    Current robot pose: x=-0.45, y=-1.48, yaw_deg=-0.2
    Known locations and distances from robot:
    - kitchen: 3.88 metres away, at (x=-1.00, y=2.35)
    - meeting_room: 1.51 metres away, at (x=0.10, y=-0.08)
    - printer: 0.05 metres away, at (x=-0.40, y=-1.47)

    Route reasoning to follow silently:
    The printer must be visited first.
    Then both kitchen and meeting_room must still be visited.
    From printer, meeting_room is closer than kitchen.
    The correct route is printer, meeting_room, kitchen.

    Example output:
    def run(robot):
        robot.say("I will visit the printer first, then the meeting room, then the kitchen.")
        robot.navigate_to("printer")
        robot.wait_until_navigate_done()
        robot.navigate_to("meeting_room")
        robot.wait_until_navigate_done()
        robot.navigate_to("kitchen")
        robot.wait_until_navigate_done()


    Example command: "Visit all rooms based on proximity from the current position."
    World context:
    Current robot pose: x=-0.93, y=1.06, yaw_deg=4.1
    Known locations and distances from robot:
    - charging_station: 2.57 metres away, at (x=1.56, y=0.45)
    - entrance: 1.79 metres away, at (x=-2.63, y=0.49)
    - home: 2.33 metres away, at (x=-2.12, y=-0.95)
    - kitchen: 1.29 metres away, at (x=-1.00, y=2.35)
    - meeting_room: 1.54 metres away, at (x=0.10, y=-0.08)
    - office: 0.04 metres away, at (x=-0.94, y=1.02)
    - printer: 2.58 metres away, at (x=-0.40, y=-1.47)

    Route reasoning to follow silently:
    The phrase based on proximity from the current position means rank locations by their initial distance from the current pose.
    The correct order is office, kitchen, meeting_room, entrance, home, charging_station, printer.

    Example output:
    def run(robot):
        robot.say("I will visit all known locations based on proximity from the current position.")
        robot.navigate_to("office")
        robot.wait_until_navigate_done()
        robot.navigate_to("kitchen")
        robot.wait_until_navigate_done()
        robot.navigate_to("meeting_room")
        robot.wait_until_navigate_done()
        robot.navigate_to("entrance")
        robot.wait_until_navigate_done()
        robot.navigate_to("home")
        robot.wait_until_navigate_done()
        robot.navigate_to("charging_station")
        robot.wait_until_navigate_done()
        robot.navigate_to("printer")
        robot.wait_until_navigate_done()


    Example command: "Move outside the map."
    World context:
    Current robot pose: x=0.00, y=0.00, yaw_deg=0.0
    Known locations and distances from robot:
    - entrance: 0.00 metres away, at (x=0.00, y=0.00)
    - office: 2.69 metres away, at (x=2.69, y=0.00)
    - kitchen: 6.08 metres away, at (x=6.08, y=0.00)

    Example output:
    def run(robot):
        robot.say("I cannot move outside the known map.")
        robot.stop()


    Example command: "Go there."
    World context:
    Current robot pose: x=0.00, y=0.00, yaw_deg=0.0
    Known locations and distances from robot:
    - entrance: 0.00 metres away, at (x=0.00, y=0.00)
    - office: 2.69 metres away, at (x=2.69, y=0.00)
    - kitchen: 6.08 metres away, at (x=6.08, y=0.00)

    Example output:
    def run(robot):
        robot.say("Please specify a valid destination.")
        robot.stop()
    """
).strip()

class LLMControlNode(Node):
    def __init__(self) -> None:
        super().__init__('llm_control')

        self.voice_topic = os.environ.get('VOICE_TOPIC', '/voice/text')
        self.model = os.environ.get('OPENAI_MODEL', 'gpt-4.1-mini')
        self.fine_tuned_model = os.environ.get('OPENAI_Finetuned_MODEL', self.model)
        self.llm_strategy = os.environ.get('LLM_STRATEGY', 'zero_shot')
        self.validation_strategy = os.environ.get('VALIDATION_STRATEGY', 'True')
        self.system_prompt = os.environ.get('POLICY_SYSTEM_PROMPT', DEFAULT_POLICY_PROMPT)
        self.robot = RobotAPI(self)

        api_key = os.environ.get('OPENAI_API_KEY', '').strip()
        if not api_key:
            raise RuntimeError('Missing OPENAI_API_KEY in environment.')

        self.client = OpenAI(api_key=api_key)

        self.create_subscription(String, self.voice_topic, self._on_voice_text, 10)

        self.get_logger().info(f'Listening for text commands on: {self.voice_topic}')
        self.get_logger().info(f'Using OpenAI model: {self.model}')
        self.get_logger().info(f'Fine-tuned model: {self.fine_tuned_model}')
        self.get_logger().info(f'LLM strategy: {self.llm_strategy}, Validation strategy: {self.validation_strategy}')

    def _on_voice_text(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return

        self.get_logger().info(f'Voice text: {text}')

        try:
            policy_code, llm_inference_latency = self._generate_policy_code(text)

            self.get_logger().info(f'Generated policy code:\n{policy_code}')

            self.get_logger().info(
                f'LLM inference latency: {llm_inference_latency:.3f} seconds'
            )

            self._execute_policy(policy_code)
        except Exception as exc:
            self.get_logger().error(f'Failed to process command: {exc}')
            self.robot.say('Sorry, I could not process that command safely.')
            self.robot.stop()

    def _select_model(self) -> str:
        if self.llm_strategy == 'fine_tuned':
            return self.fine_tuned_model
        return self.model
    
    def _build_prompt(self, user_text: str) -> list[dict[str, str]]:
        world_context = self._build_world_context()
        llm_strategy = self.llm_strategy

        if llm_strategy == 'few_shot':
            content = (
                f'{FEW_SHOT_EXAMPLES}\n\n'
                f'{world_context}\n\n'
                f'Voice command: {user_text}'
            )
            return [
                {'role': 'system', 'content': self.system_prompt},
                {'role': 'user', 'content': content},
            ]

        return [
            {'role': 'system', 'content': self.system_prompt},
            {'role': 'user', 'content': f'{world_context}\n\nVoice command: {user_text}'},
        ]
    
    def _build_world_context(self) -> str:
        pose = self.robot.current_pose
        if pose is None:
            return (
                'Current robot location: unknown\n'
                'Known waypoints: unavailable'
            )

        lines = [
            f"Current robot pose: x={pose['x']:.2f}, y={pose['y']:.2f}, yaw_deg={pose['yaw_deg']:.1f}",
            'Known locations and distances from robot:',
        ]

        for name, waypoint in sorted(self.robot.waypoints.items()):
            distance = math.dist((pose['x'], pose['y']), (waypoint['x'], waypoint['y']))
            lines.append(f'- {name}: {distance:.2f} metres away, at (x={waypoint["x"]:.2f}, y={waypoint["y"]:.2f})')

        if not self.robot.waypoints:
            lines.append('- none saved yet')

        return '\n'.join(lines)

    def _generate_policy_code(self, user_text: str) -> tuple[str, float]:
        start_time = time.perf_counter()

        response = self.client.responses.create(
            model=self._select_model(),
            input=self._build_prompt(user_text),
            temperature=0,
        )

        end_time = time.perf_counter()

        code = (response.output_text or '').strip()
        if not code:
            raise RuntimeError('Model returned empty policy code.')

        llm_inference_latency = end_time - start_time

        return code, llm_inference_latency

    def _execute_policy(self, code: str) -> None:
        tree = ast.parse(code, mode='exec')
        if self.validation_strategy.lower() == 'true':
            self._validate_policy_ast(tree)

        safe_builtins: dict[str, Any] = {
            'float': float,
            'int': int,
            'min': min,
            'max': max,
            'range': range,
            "sorted": sorted,
        }
        globals_dict = {'__builtins__': safe_builtins}
        locals_dict: dict[str, Any] = {}

        exec(compile(tree, filename='<policy>', mode='exec'), globals_dict, locals_dict)

        run_fn = locals_dict.get('run', globals_dict.get('run'))
        if not callable(run_fn):
            raise RuntimeError("Policy code must define callable function 'run(robot)'.")

        run_fn(self.robot)

    def _validate_policy_ast(self, tree: ast.AST) -> None:
        banned_nodes = (ast.Import, ast.ImportFrom, ast.With, ast.Try, ast.ClassDef, ast.Global, ast.Nonlocal)

        for node in ast.walk(tree):
            if isinstance(node, banned_nodes):
                raise RuntimeError(f'Disallowed syntax in policy: {type(node).__name__}')

            if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id in {
                'eval',
                'exec',
                '__import__',
                'open',
                'compile',
                'input',
            }:
                raise RuntimeError(f'Disallowed function in policy: {node.func.id}')


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)

    node = None
    try:
        node = LLMControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
