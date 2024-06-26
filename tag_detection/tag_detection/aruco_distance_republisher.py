import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from interfaces.msg import DetectedObject
from rclpy.qos import qos_profile_sensor_data


class DistanceRepublisher(Node):
    def __init__(self):
        super().__init__('aruco_distance_republisher')

        self.subscription = self.create_subscription(DetectedObject, '/object_detection/output', qos_profile=qos_profile_sensor_data)
        self.publisher = self.create_subscription(Float64, '/object_detection/distance_est', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.publisher.publish(msg.relative_distance)

def main(args=None):
    rclpy.init(args=args)
    subscriber = DistanceRepublisher()
    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()