import rclpy
from rclpy.node import Node

import numpy as np
from geometry_msgs.msg import PoseStamped
from ublox_ubx_msgs.msg import UBXNavRelPosNED
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64


class MinimalSubscriber(Node):

    def __init__(self):

        # Give the node a name.
        super().__init__("heading_republisher")

        # Offset is in degrees.
        # This is the offset between the GPS antennas and
        # the forward direction of the vehicle.
        self.HEADING_OFFSET = 32.27943715
        # self.HEADING_OFFSET = 0

        # Subscribe to the topic 'topic'. Callback gets called when a message is received.
        self.subscription = self.create_subscription(
            UBXNavRelPosNED,
            "/rover/ubx_nav_rel_pos_ned",
            self.listener_callback,
            qos_profile=qos_profile_sensor_data,
        )
        self.subscription  # prevent unused variable warning
        self.angle_pub_ = self.create_publisher(
            Float64, "/gps/moving_rover/heading_angle", 10
        )
        self.publisher_ = self.create_publisher(PoseStamped, "/gps/heading", 10)

        self.pose_msg = PoseStamped()

    # This callback definition simply prints an info message to the console, along with the data it received.
    def listener_callback(self, heading_msg):

        # if heading_msg.gnss_fix_ok and heading_msg.rel_pos_valid:
        rel_pos_heading = heading_msg.heading

        # Convert from 10^-5 degrees scientific notation to standard notation.
        standard_notation = rel_pos_heading * 10 ** (-5)
        # standard_notation = standard_notation + self.HEADING_OFFSET
        rel_pos_heading_rad = np.deg2rad(standard_notation)

        self.get_logger().info(str(standard_notation))
        self.angle_pub_.publish(Float64(data=float(standard_notation)))

        heading_quaternion = self.get_quaternion_from_euler(0, 0, rel_pos_heading_rad)

        self.pose_msg.header.stamp = self.get_clock().now().to_msg()
        self.pose_msg.header.frame_id = "map"
        self.pose_msg.pose.orientation.x = heading_quaternion[0]
        self.pose_msg.pose.orientation.y = heading_quaternion[1]
        self.pose_msg.pose.orientation.z = heading_quaternion[2]
        self.pose_msg.pose.orientation.w = heading_quaternion[3]

        # TODO - Add covariance info.

        self.publisher_.publish(self.pose_msg)

        print("Heading: {:.2f}".format(standard_notation))

    # Author: AutomaticAddison.com
    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.

        Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.

        Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(
            roll / 2
        ) * np.sin(pitch / 2) * np.sin(yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(
            roll / 2
        ) * np.cos(pitch / 2) * np.sin(yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(
            roll / 2
        ) * np.sin(pitch / 2) * np.cos(yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(
            roll / 2
        ) * np.sin(pitch / 2) * np.sin(yaw / 2)

        return [qx, qy, qz, qw]


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
