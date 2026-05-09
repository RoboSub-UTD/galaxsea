import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from mavros_msgs.msg import OverrideRCIn
from math import cos, sin, pi
import numpy as np


class PIDController:
    def __init__(self, kp, ki=0.0, kd=0.0, dt=0.05, output_limit=None, integral_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0.0
        self.previous_error = 0.0
        self.output_limit = output_limit
        self.integral_limit = integral_limit

    def update(self, error):
        self.integral += error * self.dt
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral)) if self.integral_limit else self.integral
        derivative = (error - self.previous_error) / self.dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        if self.output_limit is not None:
            output = max(-self.output_limit, min(self.output_limit, output))
        return output


class ThrusterController(Node):
    def __init__(self):
        super().__init__('thruster_controller')

        self.declare_parameter('max_thrust', 1000.0)
        self.declare_parameter('max_linear_velocity', 1.0)
        self.declare_parameter('max_angular_velocity', 0.25)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('sim_control', True)

        self.declare_parameter('linear_pid_kp', 250.0)
        self.declare_parameter('linear_pid_ki', 5.0)
        self.declare_parameter('linear_pid_kd', 20.0)
        self.declare_parameter('angular_pid_kp', 600.0)
        self.declare_parameter('angular_pid_ki', 10.0)
        self.declare_parameter('angular_pid_kd', 50.0)

        self.declare_parameter('l1_angle', -pi/4)
        self.declare_parameter('l2_angle', -3*pi/4)
        self.declare_parameter('r1_angle',  pi/4)
        self.declare_parameter('r2_angle',  3*pi/4)

        self.declare_parameter('l1_dist_x',  1.0)
        self.declare_parameter('l1_dist_y',  0.5)
        self.declare_parameter('l2_dist_x', -1.0)
        self.declare_parameter('l2_dist_y',  0.5)
        self.declare_parameter('r1_dist_x',  1.0)
        self.declare_parameter('r1_dist_y', -0.5)
        self.declare_parameter('r2_dist_x', -1.0)
        self.declare_parameter('r2_dist_y', -0.5)

        self.max_thrust = self.get_parameter('max_thrust').value
        self.max_linear_velocity = self.get_parameter('max_linear_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        self.control_rate = self.get_parameter('control_rate').value
        self.sim_control = self.get_parameter('sim_control').value

        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0
        self.current_linear_velocity = 0.0
        self.current_angular_velocity = 0.0

        self.linear_pid = PIDController(
            kp=self.get_parameter('linear_pid_kp').value,
            ki=self.get_parameter('linear_pid_ki').value,
            kd=self.get_parameter('linear_pid_kd').value,
            dt=1.0 / self.control_rate,
            output_limit=self.max_thrust,
            integral_limit=self.max_thrust * 0.3
        )

        self.angular_pid = PIDController(
            kp=self.get_parameter('angular_pid_kp').value,
            ki=self.get_parameter('angular_pid_ki').value,
            kd=self.get_parameter('angular_pid_kd').value,
            dt=1.0 / self.control_rate,
            output_limit=self.max_thrust,
            integral_limit=self.max_thrust * 0.2
        )

        columns = []
        for t in ['l1', 'l2', 'r1', 'r2']:
            angle  = self.get_parameter(f'{t}_angle').value
            dist_x = self.get_parameter(f'{t}_dist_x').value
            dist_y = self.get_parameter(f'{t}_dist_y').value
            fx     = cos(angle)
            fy     = sin(angle)
            torque = fx * dist_y - fy * dist_x
            columns.append([fx, fy, torque])

        A = np.array(columns).T
        self.pseudo_inv = np.linalg.pinv(A)

        self.get_logger().info(f"Allocation matrix A:\n{A}")
        self.get_logger().info(f"Pseudo-inverse:\n{self.pseudo_inv}")

        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)

        if self.sim_control:
            self.left_thruster_pub = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
            self.right_thruster_pub = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        else:
            self.actuator_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)

        self.create_timer(1.0 / self.control_rate, self.control_loop)

    def cmd_vel_callback(self, msg):
        alpha = 0.2
        target_v = max(-self.max_linear_velocity, min(self.max_linear_velocity, msg.linear.x))
        target_w = max(-self.max_angular_velocity, min(self.max_angular_velocity, msg.angular.z))
        self.target_linear_velocity = alpha * target_v + (1 - alpha) * self.target_linear_velocity
        self.target_angular_velocity = alpha * target_w + (1 - alpha) * self.target_angular_velocity

    def odom_callback(self, msg):
        self.current_linear_velocity = msg.twist.twist.linear.x
        self.current_angular_velocity = msg.twist.twist.angular.z

    def control_loop(self):
        linear_error  = self.target_linear_velocity - self.current_linear_velocity
        angular_error = self.target_angular_velocity - self.current_angular_velocity
        linear_thrust  = self.linear_pid.update(linear_error)
        angular_thrust = self.angular_pid.update(angular_error)

        if self.sim_control:
            self.publish_sim(linear_thrust, angular_thrust)
        else:
            self.publish_mavros(linear_thrust, angular_thrust)

    def publish_sim(self, linear_thrust, angular_thrust):
        left  = max(-self.max_thrust, min(self.max_thrust, linear_thrust - angular_thrust))
        right = max(-self.max_thrust, min(self.max_thrust, linear_thrust + angular_thrust))
        self.left_thruster_pub.publish(Float64(data=float(left)))
        self.right_thruster_pub.publish(Float64(data=float(right)))

    def publish_mavros(self, linear_thrust, angular_thrust):
        b = np.array([-1*linear_thrust, 0.0, -1*angular_thrust])
        #currently high pwm is high thrust onto the water
        x = self.pseudo_inv @ b

        def to_pwm(v):
            pwm = 1500 + (v / self.max_thrust) * 500
            return int(max(1000, min(2000, pwm)))

        msg = OverrideRCIn()
        msg.channels[0] = to_pwm(x[0])
        msg.channels[1] = to_pwm(x[1])
        msg.channels[2] = to_pwm(x[2])
        msg.channels[3] = to_pwm(x[3])

        self.actuator_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()