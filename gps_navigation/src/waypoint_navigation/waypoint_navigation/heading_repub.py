"""Republishes the velocity based heading to /gps/moving_rover/heading_angle
for ease-of-use with old programs."""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from ublox_ubx_msgs.msg import UBXNavRelPosNED
from rclpy.qos import qos_profile_sensor_data


class HeadingRepub(Node):
    def __init__(self):
        super().__init__("heading_repub")

        self._vel_heading_sub = self.create_subscription(
            UBXNavRelPosNED,
            "/rover/ubx_nav_rel_pos_ned",
            self.vel_callback,
            qos_profile_sensor_data,
        )

        self._gps_old_heading_publisher = self.create_publisher(
            Float64, "/gps/moving_rover/heading_angle", qos_profile_sensor_data
        )

    def vel_callback(self, msg: UBXNavRelPosNED):
        self._gps_old_heading_publisher.publish(
            Float64(data=float(msg.rel_pos_heading))
        )


def main():
    rclpy.init()

    rclpy.spin(HeadingRepub())

    rclpy.shutdown()
