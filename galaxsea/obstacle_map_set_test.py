import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import uuid

class ObstacleSetterNode(Node):
    def __init__(self):
        super().__init__('obstacle_setter')
        self.req_pub = self.create_publisher(String, '/obstacle_map/request', 10)
        self.res_sub = self.create_subscription(String, '/obstacle_map/response', self.handle_response, 10)
        self.response = None
        self.pending_req_id = None

    def handle_response(self, msg):
        data = json.loads(msg.data)
        if data.get("req_id") == self.pending_req_id:
            self.response = data

    def request_map(self, action, params=None):
        self.pending_req_id = str(uuid.uuid4())
        req = {"action": action, "req_id": self.pending_req_id}
        if params:
            req.update(params)
        self.response = None
        self.req_pub.publish(String(data=json.dumps(req)))
        while self.response is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.response

    def input_loop(self):
        print("\nObstacle Type Map Test")
        print("Format: x y label <optional_radius>")
        print("Commands: list, quit\n")

        while True:
            try:
                user_input = input("> ").strip()
                if user_input == 'quit':
                    break
                elif user_input == 'list':
                    all_obs = self.request_map("get_all")["json_data"]
                    all_obs = json.loads(all_obs)
                    if not all_obs:
                        print("No obstacles stored.")
                    else:
                        for key, obs in all_obs.items():
                            print(f"  {obs['label']} at ({obs['x']:.2f}, {obs['y']:.2f}) radius={obs['radius']:.2f}")
                else:
                    parts = user_input.split()
                    if len(parts) == 4:
                        x, y, label, radius = float(parts[0]), float(parts[1]), parts[2], float(parts[3])
                        key = self.request_map("add_obstacle", {"x": x, "y": y, "label": label, "radius": radius})["key"]
                        print(f"Added '{label}' at ({x}, {y}) radius={radius} key={key}")
                    elif len(parts) == 3:
                        x, y, label = float(parts[0]), float(parts[1]), parts[2]
                        key = self.request_map("add_obstacle", {"x": x, "y": y, "label": label})["key"]
                        print(f"Added '{label}' at ({x}, {y}) key={key}")
                    else:
                        print("Invalid format. Use: x y label  or  x y label radius")
            except ValueError:
                print("Invalid x, y, or radius — must be numbers.")
            except KeyboardInterrupt:
                break

def main():
    rclpy.init()
    node = ObstacleSetterNode()
    try:
        node.input_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()