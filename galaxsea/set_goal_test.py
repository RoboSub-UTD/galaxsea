import sys, math, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

def main():
    rclpy.init()
    node = Node('goal_sender')
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    print("Waiting for navigate_to_pose action server...")
    client.wait_for_server()

    print("\nNav2 Goal Set")
    print("Format: x y angle_degrees")
    print("Commands: quit\n")

    while rclpy.ok():
        try:
            raw = input("> ").strip()
        except EOFError:
            break

        if not raw:
            continue

        if raw.lower() == 'quit':
            break

        parts = raw.split()
        if len(parts) != 3:
            print("  Error: expected 'x y angle_degrees'")
            continue

        try:
            x, y, deg = float(parts[0]), float(parts[1]), float(parts[2])
        except ValueError:
            print("  Error: x, y, and angle must be numbers")
            continue

        yaw = math.radians(deg)

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)

        print(f"  Sending goal: x={x}, y={y}, angle={deg}°")
        client.send_goal_async(goal)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()