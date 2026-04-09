import math
import json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from std_msgs.msg import String

boat_x = 0.0
boat_y = 0.0

def odom_callback(msg: Odometry):
    global boat_x, boat_y
    boat_x = msg.pose.pose.position.x
    boat_y = msg.pose.pose.position.y

class TaskNode(Node):
    def __init__(self):
        super().__init__('task_one_node')
        self.create_subscription(Odometry, '/odometry/filtered', odom_callback, 10)
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.req_pub = self.create_publisher(String, '/obstacle_map/request', 10)
        self.res_sub = self.create_subscription(String, '/obstacle_map/response', self.handle_response, 10)
        self.response = None

    def handle_response(self, msg: String):
        self.response = json.loads(msg.data)

    def send_goal(self, x, y, yaw):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        future = self.action_client.send_goal_async(goal)
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        handle = future.result()
        if not handle.accepted:
            self.get_logger().info('Goal rejected')
            return False
        result_future = handle.get_result_async()
        self.get_logger().info("Navigating to goal")
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('Goal reached')
        return True

    def request_map(self, action, params=None):
        req = {"action": action}
        if params:
            req.update(params)
        self.response = None
        self.req_pub.publish(String(data=json.dumps(req)))
        while self.response is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.response

    def select_goal(self):
        red_res = green_res = None
        while not red_res or not red_res.get("keys") or not green_res or not green_res.get("keys"):
            red_res = self.request_map("get_closest", {"n": 1, "label": "red_pole_buoy", "exclude_used": True})
            green_res = self.request_map("get_closest", {"n": 1, "label": "green_pole_buoy", "exclude_used": True})
            if not red_res.get("keys") or not green_res.get("keys"):
                self.get_logger().info("Waiting for new red/green buoys...")
        red_key, red_x, red_y = red_res["keys"][0], red_res["xs"][0], red_res["ys"][0]
        green_key, green_x, green_y = green_res["keys"][0], green_res["xs"][0], green_res["ys"][0]
        goal_x = (red_x + green_x) / 2
        goal_y = (red_y + green_y) / 2
        perp_one = (-1 * (green_y - red_y), green_x - red_x)
        perp_two = ((green_y - red_y), -1 * (green_x - red_x))
        goal_vector = perp_one if (perp_one[0] * (goal_x - boat_x) + perp_one[1] * (goal_y - boat_y)) > (perp_two[0] * (goal_x - boat_x) + perp_two[1] * (goal_y - boat_y)) else perp_two
        goal_angle = math.atan2(goal_vector[1], goal_vector[0])
        self.request_map("mark_used", {"key": red_key})
        self.request_map("mark_used", {"key": green_key})
        self.send_goal(goal_x, goal_y, goal_angle)

def main():
    rclpy.init()
    node = TaskNode()
    while not node.action_client.wait_for_server(timeout_sec=1.0):
        rclpy.spin_once(node)
    node.get_logger().info("Selecting goal 1")
    node.select_goal()
    node.get_logger().info("Selecting goal 2")
    node.select_goal()
    node.get_logger().info("Task 1 finish")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()