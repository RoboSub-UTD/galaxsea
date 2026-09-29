import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from math import cos, sin, pi
import numpy as np

from galaxsea_interfaces.msg import ThrusterCommands


class ThrusterAllocator(Node):
    def __init__(self):
        super().__init__('thruster_allocator')

        self.declare_parameter('max_thruster_command', 1.0)
        self.declare_parameter('max_linear_velocity', 3.0)
        self.declare_parameter('max_linear_velocity_y', 3.0)
        self.declare_parameter('max_angular_velocity', 2.0)
        self.declare_parameter('control_rate', 20.0)

        self.declare_parameter('l1_angle', pi / 4)
        self.declare_parameter('r1_angle', -pi / 4)
        self.declare_parameter('l2_angle', 3 * pi / 4)
        self.declare_parameter('r2_angle', -3 * pi / 4)
        self.declare_parameter('l1_dist_x', 1.3)
        self.declare_parameter('l1_dist_y', 2.7)

        self.declare_parameter('r1_dist_x', 1.3)
        self.declare_parameter('r1_dist_y', -2.7)

        self.declare_parameter('l2_dist_x', -1.3)
        self.declare_parameter('l2_dist_y', 2.7)

        self.declare_parameter('r2_dist_x', -1.3)
        self.declare_parameter('r2_dist_y', -2.7)

        self.max_command = self.get_parameter(
            'max_thruster_command'
        ).value

        self.max_linear_velocity = self.get_parameter(
            'max_linear_velocity'
        ).value

        self.max_linear_velocity_y = self.get_parameter(
            'max_linear_velocity_y'
        ).value

        self.max_angular_velocity = self.get_parameter(
            'max_angular_velocity'
        ).value

        self.control_rate = self.get_parameter(
            'control_rate'
        ).value

        columns = []

        for t in ['l1', 'l2', 'r1', 'r2']:
            angle = self.get_parameter(
                f'{t}_angle'
            ).value

            dist_x = self.get_parameter(
                f'{t}_dist_x'
            ).value

            dist_y = self.get_parameter(
                f'{t}_dist_y'
            ).value

            fx = cos(angle)
            fy = sin(angle)

            torque = dist_x * fy - dist_y * fx

            columns.append([
                fx,
                fy,
                torque
            ])

        A = np.array(columns).T

        self.pseudo_inv = np.linalg.pinv(A)

        self.target_linear_x = 0.0
        self.target_linear_y = 0.0
        self.target_angular = 0.0

        self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.thruster_pub = self.create_publisher(
            ThrusterCommands,
            '/thruster_commands',
            10
        )

        self.create_timer(
            1.0 / self.control_rate,
            self.control_loop
        )

    def cmd_vel_callback(self, msg):
        self.target_linear_x = max(
            -self.max_linear_velocity,
            min(
                self.max_linear_velocity,
                msg.linear.x 
            )
        )

        self.target_linear_y = max(
            -self.max_linear_velocity_y,
            min(
                self.max_linear_velocity_y,
                msg.linear.y
            )
        )

        self.target_angular = max(
            -self.max_angular_velocity,
            min(
                self.max_angular_velocity,
                msg.angular.z 
            )
        )

    def control_loop(self):
        b = np.array([
            self.target_linear_y,
            -self.target_linear_x,
            self.target_angular
        ])

        x = self.pseudo_inv @ b

        peak = np.max(np.abs(x))

        if peak > self.max_command:
            x *= self.max_command / peak

        msg = ThrusterCommands()

        msg.front_left = float(x[0])
        msg.back_left = float(x[1])
        msg.front_right = float(x[2])
        msg.back_right = float(x[3])

        self.thruster_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = ThrusterAllocator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
