# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from interfaces.msg import DetectedObject
from std_msgs.msg import Float64MultiArray

from rclpy.qos import qos_profile_sensor_data

class MinimalSubscriber(Node):

    def __init__(self):

        # Give the node a name.
        super().__init__('minimal_subscriber')

        # Subscribe to the topic 'topic'. Callback gets called when a message is received.
        self.subscription = self.create_subscription(
            DetectedObject,
            '/object_detection/distance_est',
            self.listener_callback,
            qos_profile=qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(Float64MultiArray, '/object_detection/navigation_velocity', 10)

        self.velocity_result = Float64MultiArray()

    # This callback definition simply prints an info message to the console, along with the data it received. 
    def listener_callback(self, msg):

        x2, y2 = msg.top_right.x, msg.top_right.y
        x1, y1 = msg.bottom_left.x, msg.bottom_left.y

        x_midpoint = (x2 + x1) / 2
        image_size = (1920, 1080)
        left_bound = int(.39583333 * image_size[0])
        right_bound = int(.60416667 * image_size[0])

        vel_left = 0
        vel_right = 0

        if x_midpoint < left_bound:
            vel_right = 1
        elif x_midpoint > right_bound:
            vel_left = 1
        
        self.velocity_result.data = [float(vel_left), float(vel_right)]

        self.publisher_.publish(self.velocity_result)



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
