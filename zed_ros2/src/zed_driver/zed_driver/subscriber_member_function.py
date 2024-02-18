




# TODO - This will be used for fixing the tf tree issue for navsat_transform_node

# Test:
# 1. Sub to  odom filtered local and change the frame_id or child frame to map
# 2. OR sub to base/fix and change the frame_id or child frame to map




import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry


class MinimalSubscriber(Node):

    def __init__(self):

        # Give the node a name.
        super().__init__('odom_republisher')

        # Subscribe to the topic 'topic'. Callback gets called when a message is received.
        self.subscription = self.create_subscription(
            Odometry,
            '/zed/zed_node/odom',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(Odometry, '/zed/custom/odom_republisher', 10)



    # This callback definition simply prints an info message to the console, along with the data it received. 
    def listener_callback(self, msg):
        msg.header.frame_id = "map"
        # msg.child_frame_id = "zed_camera_link"
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
