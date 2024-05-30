from serial import Serial

import rclpy
from rclpy.node import Node
from rtcm_msgs.msg import Message as ROS_RTCM_Message

from pyrtcm import RTCMReader

import numpy as np


class MinimalPublisher(Node):

    def __init__(self):

        # Give the node a name.
        super().__init__("rtcm_publisher")

        self.stream = None

        self.try_connect_serial()

        if not self.rtcm_reader:
            self.get_logger().error("Could not connect to an open serial device!")
            exit(1)

        # Specify data type and topic name. Specify queue size (limit amount of queued messages)
        self.publisher_ = self.create_publisher(ROS_RTCM_Message, "/base/gps/rtcm3", 10)

        # Create a timer that will call the 'timer_callback' function every timer_period second.
        timer_period = 1 / 5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def __del__(self):
        if self.stream != None:
            self.stream.close()

    def try_connect_serial(self):
        self.stream = None

        # Try to iterate through the three USB ports in use
        # Assume that ublox_gnss modules have already taken control of other ports
        for i in range(3):
            try:
                self.stream = Serial(f"/dev/ttyACM{i}", 460800, timeout=3)
                self.stream.write(0)
            except IOError:
                pass

        if self.stream is not None:
            self.get_logger().info(f"Connected to serial device /dev/ttyACM{i}")
        else:
            return

        self.rtcm_reader = RTCMReader(self.stream)

    def timer_callback(self):

        self.msg = ROS_RTCM_Message()
        try:
            (raw_data, parsed_data) = self.rtcm_reader.read()
        except IOError:
            self.get_logger().warning(
                f"Could not read from serial device {self.stream}! Attempting to reconnect..."
            )
            self.try_connect_serial()
            return

        if raw_data is None:
            return

        np_arr = np.frombuffer(raw_data, dtype=np.uint8)

        if np_arr is None:
            return

        # self.get_logger().info(str(np_arr.tolist()))

        self.msg.message = np_arr.tolist()
        self.msg.header.stamp = self.get_clock().now().to_msg()

        self.publisher_.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
