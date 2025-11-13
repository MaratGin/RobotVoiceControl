import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from flask import Flask, request, jsonify
import threading
import json
import time

app = Flask(__name__)
ros_node = None

@app.route('/bridge', methods=['POST'])
def receive_command():
    
    data = request.get_json()
    command = llm_to_ros_command(llm_json=data)

    twist = TwistStamped()
    print(command)

    twist.twist.linear.x = command["linear"]["x"] * 5
    print("SEND TO NODE")
    print(ros_node)
    for i in range(10):
        ros_node.publisher.publish(twist)
        time.sleep(0.3)
    ros_node.get_logger().info(f"Received command: {command}")
    return jsonify({"status": "ok", "sent": command})

def llm_to_ros_command(llm_json: str) -> dict:
    """
    Преобразует JSON-ответ LLM в JSON-команду для робота.
    Возвращает Python-словарь (готов для json.dumps() и отправки по сети).
    """
    try:
        print(llm_json)
        print(type(llm_json))
        data = json.dumps(llm_json)
        data = json.loads(data)
        response = data.get("response", "")
        parts = response.split(";")
        print(1)
        ros_cmd = {
            "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
        }
        print(2)

        if parts[0] == "command" and parts[1] == "stop":
            ros_cmd["linear"]["x"] = 0.0
            ros_cmd["angular"]["z"] = 0.0

        elif parts[0] == "movement" and len(parts) == 3:
            print(3)
            direction = parts[1]
            speed = float(parts[2]) * 5

            if direction == "forward":
                ros_cmd["linear"]["x"] = speed
            elif direction == "backward":
                ros_cmd["linear"]["x"] = -speed

        else:
            ros_cmd["error"] = f"Unknown command: {response}"

        return ros_cmd
    except Exception as e:
        return {"error": str(e)}

class RosBridgeNode(Node):
    def __init__(self):
        super().__init__('flask_ros_bridge')
        print("CREATE PUBLISHER")
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.get_logger().info("ROS bridge node started")

def flask_thread():
    app.run(host='0.0.0.0', port=5001)

def main(args=None):
    global ros_node
    rclpy.init(args=args)
    ros_node = RosBridgeNode()

    # Flask в отдельном потоке
    flask_t = threading.Thread(target=flask_thread, daemon=True)
    flask_t.start()
    print("PUBLISH")

    rclpy.spin(ros_node)
    print("PUBLISH")

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

"""
ros2 topic pub --rate 10 /cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: -10.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
"""