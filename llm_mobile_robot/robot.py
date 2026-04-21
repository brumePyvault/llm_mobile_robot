class RobotAPI:
    """
    Restricted robot control API exposed to generated policies.
    Only methods defined here are allowed to be used by the policy code.
    """

    def __init__(self, node: Node):
        self.node = node

    def say(self, text: str):
        self.node.get_logger().info(f"[ROBOT SAY] {text}")

    def navigate_to(self, location: str):
        # Replace with Nav2 action call
        self.node.get_logger().info(f"[NAVIGATE] Going to: {location}")

    def stop(self):
        # Replace with robot stop logic
        self.node.get_logger().info("[STOP] Robot stopping")

    def save_waypoint(self, name: str):
        # Replace with waypoint persistence logic
        self.node.get_logger().info(f"[WAYPOINT] Saving current pose as: {name}")

    def drive(self, linear_x: float, angular_z: float, duration_s: float):
        # Replace with Twist publisher or motion command logic
        self.node.get_logger().info(
            f"[DRIVE] linear_x={linear_x}, angular_z={angular_z}, duration_s={duration_s}"
        )
        time.sleep(max(0.0, min(duration_s, 10.0)))

    def come_back(self):
        # Replace with actual “return to base/home” behaviour
        self.node.get_logger().info("[RETURN] Returning to base")

