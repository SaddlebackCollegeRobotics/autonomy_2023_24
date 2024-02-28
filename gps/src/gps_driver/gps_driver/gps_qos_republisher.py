import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import NavSatFix
from rclpy.qos import qos_profile_sensor_data

class  MinimalSubscriber(Node):

    def __init__(self):

        # Give the node a name.
        super().__init__('gps_qos_republisher')

        self.subscription = self.create_subscription(
            NavSatFix,
            '/base/fix',
            self.listener_callback,
            qos_profile=qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(NavSatFix, '/base/fix_qos', 10)

    def listener_callback(self, msg):
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
