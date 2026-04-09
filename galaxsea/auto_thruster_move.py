import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

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
        if self.integral_limit is not None:
            self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
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
        self.declare_parameter('linear_pid_kp', 250.0)
        self.declare_parameter('linear_pid_ki', 5.0)
        self.declare_parameter('linear_pid_kd', 20.0)
        self.declare_parameter('angular_pid_kp', 600.0)
        self.declare_parameter('angular_pid_ki', 10.0)
        self.declare_parameter('angular_pid_kd', 50.0)

        self.max_thrust = self.get_parameter('max_thrust').value
        self.max_linear_velocity = self.get_parameter('max_linear_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        self.control_rate = self.get_parameter('control_rate').value

        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0
        self.current_linear_velocity = 0.0
        self.current_angular_velocity = 0.0

        self.linear_pid = PIDController(
            kp=self.get_parameter('linear_pid_kp').value,
            ki=self.get_parameter('linear_pid_ki').value,
            kd=self.get_parameter('linear_pid_kd').value,
            dt=1.0/self.control_rate,
            output_limit=self.max_thrust,
            integral_limit=self.max_thrust*0.3
        )

        self.angular_pid = PIDController(
            kp=self.get_parameter('angular_pid_kp').value,
            ki=self.get_parameter('angular_pid_ki').value,
            kd=self.get_parameter('angular_pid_kd').value,
            dt=1.0/self.control_rate,
            output_limit=self.max_thrust,
            integral_limit=self.max_thrust*0.2
        )

        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.left_thruster_pub = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.right_thruster_pub = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        self.create_timer(1.0/self.control_rate, self.control_loop)

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
        linear_error = self.target_linear_velocity - self.current_linear_velocity
        angular_error = self.target_angular_velocity - self.current_angular_velocity
        linear_thrust = self.linear_pid.update(linear_error)
        angular_thrust = self.angular_pid.update(angular_error)
        left_thruster = max(-self.max_thrust, min(self.max_thrust, linear_thrust - angular_thrust))
        right_thruster = max(-self.max_thrust, min(self.max_thrust, linear_thrust + angular_thrust))
        self.left_thruster_pub.publish(Float64(data=float(left_thruster)))
        self.right_thruster_pub.publish(Float64(data=float(right_thruster)))

def main(args=None):
    rclpy.init(args=args)
    node = ThrusterController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()