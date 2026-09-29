import curses
from curses import wrapper

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ControllerPublisher(Node):
    def __init__(self):
        super().__init__("boat_controller_pub")
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def controller(self, stdscr):
        msg = Twist()

        stdscr.clear()
        stdscr.addstr(0, 0, "-----------------------------")
        stdscr.addstr(1, 0, "Moving around:")
        stdscr.addstr(2, 0, "Control boat")
        stdscr.addstr(3, 0, "   q   w   e  ")
        stdscr.addstr(4, 0, "   a   s   d  ")
        stdscr.addstr(5, 0, "       x      ")
        stdscr.addstr(6, 0, " ")
        stdscr.addstr(7, 0, "w/x/a/d: increase linear velocity")
        stdscr.addstr(8, 0, "q/e: increase angular velocity")
        stdscr.addstr(9, 0, "s: stop")
        stdscr.addstr(10, 0, " ")
        stdscr.addstr(11, 0, "Press Ctrl C to exit")
        stdscr.refresh()
        output_win = curses.newwin(1, 80, 12, 0)

        def show_status():
            output_win.clear()
            output_win.addstr(
                0, 0,
                f"currently:   Linear_x: {msg.linear.x:.2f}  "
                f"Linear_y: {msg.linear.y:.2f}  "
                f"Angular: {msg.angular.z:.2f}",
            )
            output_win.refresh()

        show_status()

        while True:
            try:
                key = output_win.getkey()
            except KeyboardInterrupt:
                break
            except curses.error:
                continue

            match key:
                case "w":
                    msg.linear.x += 1.0
                case "x":
                    msg.linear.x -= 1.0
                case "a":
                    msg.linear.y += 1.0
                case "d":
                    msg.linear.y -= 1.0
                case "q":
                    msg.angular.z -= 0.5
                case "e":
                    msg.angular.z += 0.5
                case "s":
                    msg.linear.x = msg.linear.y = msg.angular.z = 0.0

            self.cmd_vel_pub.publish(msg)
            show_status()

        self.cmd_vel_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    ctrl_pub = ControllerPublisher()
    try:
        wrapper(ctrl_pub.controller)
    finally:
        ctrl_pub.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()