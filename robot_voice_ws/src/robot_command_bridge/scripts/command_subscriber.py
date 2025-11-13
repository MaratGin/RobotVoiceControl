# robot_command_bridge/command_subscriber.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import json


class LLMCommandToCmdVel(Node):
    def __init__(self):
        super().__init__('llm_command_to_cmd_vel')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_unstamped', 10)
        self.get_logger().info("LLM Command to cmd_vel bridge is ready (mock mode).")

        self.run_mock_tests()

    def process_llm_response(self, response_str: str):
        """Parse 'response' string like 'movement;forward;1.0' or 'command;stop'."""
        parts = [p.strip() for p in response_str.split(';')]
        cmd = Twist()

        if len(parts) < 1:
            self.get_logger().warn("Empty command received.")
            return

        if parts[0] == "movement" and len(parts) == 3:
            _, direction, speed_str = parts
            try:
                speed = float(speed_str)
                if not (0.5 <= abs(speed) <= 2.0):
                    self.get_logger().warn(f"Speed {speed} out of expected range [0.5, 2.0]")
            except ValueError:
                self.get_logger().error(f"Invalid speed: {speed_str}")
                return

            if direction == "forward":
                cmd.linear.x = speed
            elif direction == "backward":
                cmd.linear.x = -speed
            else:
                self.get_logger().warn(f"Unknown direction: {direction}")
                return

        elif parts[0] == "command" and len(parts) == 2:
            if parts[1] == "stop":
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            else:
                self.get_logger().warn(f"Unknown command: {parts[1]}")
                return
        else:
            self.get_logger().error(f"Unrecognized command format: '{response_str}'")
            return

        self.publisher_.publish(cmd)
        self.get_logger().info(f"Published cmd_vel: linear.x={cmd.linear.x:.2f}")

    def run_mock_tests(self):
        mock_responses = [
            '{"response": "movement;forward;1.5"}',
            '{"response": "movement;backward;0.7"}',
            '{"response": "command;stop"}',
        ]

        import time
        for mock_json in mock_responses:
            try:
                data = json.loads(mock_json)
                response = data.get("response", "")
                self.get_logger().info(f"Processing mock: {response}")
                self.process_llm_response(response)
                time.sleep(2)
            except Exception as e:
                self.get_logger().error(f"Mock processing error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = LLMCommandToCmdVel()
    try:
        rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()