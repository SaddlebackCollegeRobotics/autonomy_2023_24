from typing import NamedTuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped
from std_msgs.msg import Float64MultiArray, Header

from .diff_drive_kinematics import diff_drive_ik, diff_drive_fk, rpm_to_rad


class DiffDriveInfo(NamedTuple):
    wheel_rad: float  # [m]
    wheel_sep: float  # [m]
    max_wheel_speed: float  # [rad/s]


class RobotNode(Node):
    """All variables using SI units"""

    def __init__(self, robot_info: DiffDriveInfo,):
       
        super().__init__('robot_node')  # type: ignore

        print('initializing...')

        # setup math for kinematics and odometry
        self.radius = robot_info.wheel_rad
        self.separation = robot_info.wheel_sep
        self.max_wheel_speed = robot_info.max_wheel_speed

        fraction = 2/3
        self.max_linear_speed, _ = diff_drive_fk(self.max_wheel_speed*fraction,
                                               self.max_wheel_speed*fraction,
                                               self.radius,
                                               self.separation)
        _, self.max_angular_speed = diff_drive_fk(-self.max_wheel_speed*fraction,
                                                  self.max_wheel_speed*fraction,
                                                  self.radius,
                                                  self.separation)

        self.create_subscription(
            Twist, '/cmd_vel', self.run_motors, 10)
        self.create_subscription(
            Float64MultiArray, '/drive/wheel_velocity_estimates', self.calc_drive_fk, 10)

        self._wheel_vel_pub = self.create_publisher(
            Float64MultiArray, '/control/drive_control_input', 10)
        self._wheel_odometry_feedback = self.create_publisher(
            TwistWithCovarianceStamped, '/odometry/wheel_feedback', 10)

        print('ready!')

    def calc_drive_fk(self, side_est: Float64MultiArray) -> None:
        left_side_est, right_side_est = side_est.data

        linear_vel_x, angular_vel_z = diff_drive_fk(
            left_side_est, right_side_est, self.radius, self.separation
        )

        twist_msg = Twist()
        msg_header = Header()
        twist_msg.linear.x = linear_vel_x
        twist_msg.angular.z = angular_vel_z
        msg_header.stamp = self.get_clock().now().to_msg()
        msg_header.frame_id = "drive_vel_link"

        twist_msg_with_covariance = TwistWithCovarianceStamped(
            twist=twist_msg, header=msg_header
        )
        twist_msg_with_covariance.covariance[0] = -1 # Disable covariance

        self._wheel_odometry_feedback.publish(twist_msg_with_covariance)

    def run_motors(self, twist_msg) -> None:
        # transform linear and angular commands into percents of wheel speeds
        linear = twist_msg.linear.x * self.max_linear_speed  
        angular = twist_msg.angular.z * self.max_angular_speed 
        left_wheel_vel, right_wheel_vel = diff_drive_ik(
            linear, -angular, self.radius, self.separation)

        left_percent_max_speed = left_wheel_vel / self.max_wheel_speed * 100
        right_percent_max_speed = right_wheel_vel / self.max_wheel_speed * 100

        left_vel = left_percent_max_speed / 100
        right_vel = right_percent_max_speed / 100

        # # command motors to spin with set percent of max speed
        # self.motors.start(-left_percent_max_speed, right_percent_max_speed)
        print(f'Driving with: \
              l={left_vel}, \
              r={right_vel}')

        # publish wheel velocities for odometry
        wheel_vels = Float64MultiArray()
        # wheel_vels.data = [
        #     self.max_wheel_speed * left_percent_max_speed / 100,  # rad/s
        #     self.max_wheel_speed * right_percent_max_speed / 100  # rad/s
        # ]
        wheel_vels.data = [
            left_vel,
            right_vel
        ]
        self._wheel_vel_pub.publish(wheel_vels)


def main(args=None):
    
    # TODO: Ensure correctness of measured values
    robot_info = DiffDriveInfo(
        wheel_rad=0.174,  # [m]
        wheel_sep=0.785,  # [m]
        max_wheel_speed=rpm_to_rad(4200.0) # [rad/s]
    )  

    rclpy.init(args=args)

    node = RobotNode(robot_info)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()


if __name__ == "__main__":
    main()
