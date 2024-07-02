import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import NavSatFix, NavSatStatus
from ublox_ubx_msgs.msg import UBXNavRelPosNED

from rclpy.qos import qos_profile_sensor_data


class MinimalPublisher(Node):

    def __init__(self):

        super().__init__("mock_data_publisher")

        self.waypoint_publisher = self.create_publisher(
            Float64MultiArray, "/autonomy/waypoints", 10
        )
        self.gps_pos_publisher = self.create_publisher(
            NavSatFix, "/base/fix", qos_profile_sensor_data
        )
        self.gps_heading_publisher = self.create_publisher(
            UBXNavRelPosNED, "/rover/ubx_nav_rel_pos_ned", qos_profile_sensor_data
        )

        # test data

        self.gps_pos_list = [
            # (2.0, 2.0),
            # (2.0, -2.0),
            (-2.0, -2.0),
            # (-2.0, 2.0),
            # (0.0, 0.0),
        ]

        self.heading_list = [
            0,
            45,
            90,
            135,
            180,
            225,
            270,
            315,
            360,
        ]

        self.target_waypoint = [0.0, 0.0]

        self.gps_pos_msg = NavSatFix()
        self.gps_heading_msg = UBXNavRelPosNED()
        self.waypoints_msg = Float64MultiArray()

        self.gps_pos_list_index = 0
        self.heading_list_index = 0

        self.publish_waypoints()

        timer_period = 1 / 5
        self.gps_pos_timer = self.create_timer(timer_period, self.publish_gps_pos)
        self.gps_heading_timer = self.create_timer(1, self.publish_gps_heading)

    def publish_gps_pos(self):

        current_pos = self.gps_pos_list[self.gps_pos_list_index]
        self.gps_pos_list_index += 1

        if self.gps_pos_list_index >= len(self.gps_pos_list):
            self.gps_pos_list_index = 0

        self.gps_pos_msg.latitude = current_pos[0]
        self.gps_pos_msg.longitude = current_pos[1]

        self.gps_pos_msg.status.status = NavSatStatus.STATUS_FIX
        self.gps_pos_publisher.publish(self.gps_pos_msg)

    def publish_gps_heading(self):

        current_heading = self.heading_list[self.heading_list_index]
        self.heading_list_index += 1

        if self.heading_list_index >= len(self.heading_list):
            self.heading_list_index = 0

        self.gps_heading_msg.rel_pos_heading = int(current_heading)

        self.gps_heading_publisher.publish(self.gps_heading_msg)

    def publish_waypoints(self):
        self.waypoints_msg.data = self.target_waypoint
        self.waypoint_publisher.publish(self.waypoints_msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
