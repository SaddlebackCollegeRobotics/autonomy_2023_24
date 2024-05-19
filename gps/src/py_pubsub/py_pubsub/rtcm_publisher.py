from serial import Serial, SerialException

import rclpy
from rclpy.node import Node
from rtcm_msgs.msg import Message as RTCMMessage

from pyrtcm import RTCMReader

import numpy as np

class MinimalPublisher(Node):

    def __init__(self):

        # Give the node a name.
        super().__init__('rtcm_publisher')

        self.dev_path = '/dev/ttyACM0'

        try:
            self.serial = Serial(self.dev_path, 460800, timeout=3) # TODO - double check baudrate
        except SerialException:
            print("Error: Cannot find GPS device on:", self.dev_path)
            exit(1)
        
        self.rtcm_reader = RTCMReader(self.serial)

        # Specify data type and topic name. Specify queue size (limit amount of queued messages)
        self.publisher_ = self.create_publisher(RTCMMessage, '/gps/static_base/rtcm_correction', 10)

        self.msg = RTCMMessage()

        # Create a timer that will call the 'timer_callback' function every timer_period second.
        timer_period = 1/2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        
        (raw_data, parsed_data) = self.rtcm_reader.read()

        if raw_data is None:
            return

        print(raw_data)

        np_arr = np.frombuffer(raw_data, dtype=np.uint8) # Does this need to be uint8?

        if np_arr is None:
            return

        self.msg.message = np_arr.tolist()
        self.msg.header.stamp = self.get_clock().now().to_msg()

        self.publisher_.publish(self.msg)
        

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
