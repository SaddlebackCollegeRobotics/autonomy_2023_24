from serial import Serial

import rclpy
from rclpy.node import Node
from rtcm_msgs.msg import Message as ROS_RTCM_Message

from pyrtcm import RTCMReader

import numpy as np


class MinimalPublisher(Node):

    def __init__(self):

        # Give the node a name.
        super().__init__('minimal_publisher')

        self.stream = Serial('/dev/ttyACM0', 460800, timeout=3)
        self.rtcm_reader = RTCMReader(self.stream)

        # Specify data type and topic name. Specify queue size (limit amount of queued messages)
        self.publisher_ = self.create_publisher(ROS_RTCM_Message, '/gps/rtcm', 10)

        # Create a timer that will call the 'timer_callback' function every timer_period second.
        timer_period = 1/5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def __del__(self):
        self.stream.close()

    def timer_callback(self):
        
        self.msg = ROS_RTCM_Message()
        (raw_data, parsed_data) = self.rtcm_reader.read()

        if raw_data is None:
            return

        np_arr = np.frombuffer(raw_data, dtype=np.uint8)

        if np_arr is None:
            return

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


if __name__ == '__main__':
    main()
