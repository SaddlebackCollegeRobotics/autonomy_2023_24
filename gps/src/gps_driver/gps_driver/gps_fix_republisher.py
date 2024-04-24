from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from rclpy.qos import qos_profile_sensor_data
import rclpy

class MinimalSubscriber(Node):

    def __init__(self):

        # Give the node a name.
        super().__init__('gps_fix_republisher')

        # Subscribe to the topic 'topic'. Callback gets called when a message is received.
        self.subscription = self.create_subscription(
            NavSatFix,
            '/base/fix',
            self.listener_callback,
            qos_profile=qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

        self.pub_qos_sensor = self.create_publisher(NavSatFix, '/base/fix_qos_sensor', qos_profile=qos_profile_sensor_data)
        self.pub_qos_reliable = self.create_publisher(NavSatFix, '/base/fix_qos_reliable', 10)


    # This callback definition simply prints an info message to the console, along with the data it received. 
    def listener_callback(self, msg):

        msg.header.frame_id = "gnss_moving_base_link"
        
        self.pub_qos_sensor.publish(msg)
        self.pub_qos_reliable.publish(msg)


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
