import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Float64MultiArray, String, Bool
from ublox_ubx_msgs.msg import UBXNavVelNED
from sensor_msgs.msg import NavSatFix, NavSatStatus

from math import atan2, degrees, copysign

from rclpy.qos import qos_profile_sensor_data

from geopy.distance import geodesic
from enum import Enum
from time import sleep


class Direction(Enum):
    Left = 0
    Right = 1


class MinimalPublisher(Node):

    FORWARD_SPEED = 1.0
    TURN_SPEED = 0.3

    TARGET_DIST_THRESH = 1  # meters

    def __init__(self):

        super().__init__("waypoint_navigator")

        self.drive_input_publisher = self.create_publisher(
            Float64MultiArray, "/drive/control_input", 10
        )

        self.waypoint_subscriber = self.create_subscription(
            Float64MultiArray, "/autonomy/waypoints", self.waypoint_callback, 10
        )
        self.waypoint_subscriber  # prevent unused variable warning

        self.gps_fix_subscriber = self.create_subscription(
            NavSatFix, "/base/fix", self.gps_fix_callback, qos_profile_sensor_data
        )
        self.gps_fix_subscriber  # prevent unused variable warning

        self.start_obj_publisher = self.create_publisher(
            Bool, "/object_detection/active", 10
        )

        self.gps_heading_subscriber = self.create_subscription(
            UBXNavVelNED,
            "/rover/ubx_nav_vel_ned",
            self.gps_heading_callback,
            qos_profile=qos_profile_sensor_data,
        )
        self.led_pub = self.create_publisher(String, "/led_pub", 10)

        self.drive_input_msg = Float64MultiArray()

        self.current_heading: float = None
        self.current_position: tuple = None

        self.gps_fix_ok: bool = False

        self.waypoint_list: list = []
        self.waypoint_num = 1

        self.heading_turn_threshold: int = 30  # degrees

        self.previous_position = None
        self.est_heading = 0

        timer_period = 1 / 10
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.timer.cancel()

    def gps_fix_callback(self, msg: NavSatFix):

        if msg.status.status == NavSatStatus.STATUS_NO_FIX:
            # stop drive sys
            self.gps_fix_ok = False
            print("BAD FIX!")
        else:
            self.gps_fix_ok = True
            self.current_position = (msg.latitude, msg.longitude)

    def gps_heading_callback(self, msg):
        # stops publishing when cannot find heading
        self.current_heading = msg.heading / (10 ** 5)

    def waypoint_callback(self, msg: Float64MultiArray):

        self.timer.cancel()

        while self.timer.is_canceled() == False:
            continue

        self.led_pub.publish(String(data="red"))

        self.drive_input_msg.data = [1.0, 1.0]
        self.drive_input_publisher.publish(self.drive_input_msg)

        sleep(2)

        self.drive_input_msg.data = [0.0, 0.0]
        self.drive_input_publisher.publish(self.drive_input_msg)

        self.current_position = None
        self.current_heading = None

        self.waypoint_list = msg.data
        self.waypoint_num = 1

        print("Added new waypoints!")
        print(
            f"Navigating to waypoint #{self.waypoint_num}: {self.waypoint_list[0]}, {self.waypoint_list[1]}"
        )

        self.timer.reset()

    def timer_callback(self):

        if (
            len(self.waypoint_list) == 0
            or self.gps_fix_ok == False
            or self.current_position == None
            or self.current_heading == None
        ):
            # print(
            #     f"""{len(self.waypoint_list)=}
            # {self.gps_fix_ok}
            # {self.current_position}
            # {self.current_heading}"""
            # )
            return

        current_position = self.current_position
        current_heading = self.current_heading

        x = self.waypoint_list[0] - current_position[0]  # latitude diff
        y = self.waypoint_list[1] - current_position[1]  # longitude diff

        print(f"{self.current_position[0]=} {self.current_position[1]=}")
        print(f"{self.waypoint_list[0]=} {self.waypoint_list[1]=}")
        print(f"{x=} {y=}")

        target_heading = (270 - degrees(atan2(x, y))) % 360
        ## target_heading += 90
        ## target_heading %= 360


        target_distance = geodesic(
            (current_position[0], current_position[1]), tuple(self.waypoint_list[:2])
        ).meters

        print(
            f"Position: {current_position[0]}, {current_position[1]} \
                Heading: {current_heading} \
                Target Heading: {target_heading} \
                Target Distance: {target_distance}"
        )

        movement_output = [0.0, 0.0]

        if target_distance > self.TARGET_DIST_THRESH:
            movement_output = [self.FORWARD_SPEED, self.FORWARD_SPEED]

            if abs(target_heading - current_heading) > self.heading_turn_threshold:

                heading_delta = abs(current_heading - target_heading)

                if current_heading < target_heading:
                    if heading_delta < 180:
                        rotation_dir = Direction.Right
                    else:
                        rotation_dir = Direction.Left
                else:
                    if heading_delta < 180:
                        rotation_dir = Direction.Left
                    else:
                        rotation_dir = Direction.Right

                if rotation_dir == Direction.Right:
                    # Turn Left: left side slower than right
                    print(f"Turning RIGHT {heading_delta=} {target_heading=} {current_heading=}")
                    movement_output = [self.TURN_SPEED, self.FORWARD_SPEED]
                else:
                    print(f"Turning LEFT {heading_delta=} {target_heading=} {current_heading=}")
                    # Turn right: right side slower than left
                    movement_output = [self.FORWARD_SPEED, self.TURN_SPEED]
            else:
                print("FORWARD")
        else:

            if len(self.waypoint_list) > 0:

                self.waypoint_list.pop(0)
                self.waypoint_list.pop(0)

                # Start the aruco tag detection behavior node (spiral search -> stop at tag)
                self.start_obj_publisher.publish(Bool(data=True))

                if len(self.waypoint_list) > 0:
                    self.waypoint_num += 1
                    print(
                        f"Navigating to waypoint #{self.waypoint_num}: {self.waypoint_list[0]}, {self.waypoint_list[1]}"
                    )
                else:
                    print("Finished Navigation!")

        self.drive_input_msg.data = movement_output
        self.drive_input_publisher.publish(self.drive_input_msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()