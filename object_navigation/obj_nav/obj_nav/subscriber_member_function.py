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

import rclpy
from threading import Thread
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray, String
from tag_interface.msg import Tag as TagData

from time import perf_counter, sleep
from enum import Enum
from std_msgs.msg import Bool

class MovementState(Enum):
    NONE = 0
    FOLLOW = 1
    SPIRAL = 2

TARGET_DIST = 300.0             # cm
DEAD_ZONE = 300                 # pixel width
WIDTH, HEIGHT = (2208, 1242)

class MinimalSubscriber(Node):

    STOP = [0.0, 0.0]
    FORWARD = [1.0, 1.0]
    SPIN = [-1.0, 1.0]
    MOVE_LEFT = [0.2, 1.0]
    MOVE_RIGHT = [1.0, 0.2]

    # Frequency at which this node will publish messages, in Hz
    SPIRAL_LOOP_RATE: float = 15

    # Speed to rotate at, rad/s
    # This is for IN-PLACE rotation
    SPIRAL_ROTATION_SPEED: float = 0.5

    def __init__(self):

        # Give the node a name.
        super().__init__('minimal_subscriber')

        self.state = MovementState.NONE

        # Subscribe to the topic 'topic'. Callback gets called when a message is received.
        self.subscription = self.create_subscription(
            DetectedObject,
            '/object_detection/distance_est',
            self.follow_callback,
            qos_profile=qos_profile_sensor_data)
        self.start_sub = self.create_subscription(
            Bool,
            '/object_detection/active',
            self.set_active,
            10)

        self.led_pub = self.create_publisher(String, '/led_pub', 10)

        self.drive_publisher = self.create_publisher(Float64MultiArray, '/drive/control_input', 10)
        self.velocity_result = Float64MultiArray()

        self._turn_timer = None # To be set to timer instances when we want non-blocking callbacks
        self._spiral_timer = None


    # This callback definition simply prints an info message to the console, along with the data it received. 
    def follow_callback(self, msg):
        if self.state == MovementState.NONE:
            self.get_logger().info("NONE")
            return
        self.get_logger().info("HERE")

        x2, y2 = msg.top_right.x, msg.top_right.y
        x1, y1 = msg.bottom_left.x, msg.bottom_left.y

        x_midpoint = (x2 + x1) / 2
        # image_size = (1920, 1080)
        # left_bound = int(.39583333 * image_size[0])
        # right_bound = int(.60416667 * image_size[0])

        # vel_left = 0
        # vel_right = 0

        # if x_midpoint < left_bound:
        #     vel_right = 1
        # elif x_midpoint > right_bound:
        #     vel_left = 1
        
        # self.velocity_result.data = [float(vel_left), float(vel_right)]


        # self.get_logger().info(f"Setting vel = {self.velocity_result.data}")
        # self.publisher_.publish(self.velocity_result)

        ################################################################
        # update velocity
        self.get_logger().info(f"{x_midpoint=}")
        if x_midpoint > WIDTH / 2 - DEAD_ZONE and x_midpoint < WIDTH / 2 + DEAD_ZONE:
            self.velocity_result.data = MinimalSubscriber.FORWARD
        elif x_midpoint <= WIDTH / 2:
            self.velocity_result.data = MinimalSubscriber.MOVE_LEFT
        else:
            self.velocity_result.data = MinimalSubscriber.MOVE_RIGHT

        # If we've reached the goal, stop 
        if msg.relative_distance <= TARGET_DIST:
            self.velocity_result.data = MinimalSubscriber.STOP
            self.led_pub.publish(String(data="green"))
            self.state = MovementState.NONE
            # TODO: Maybe stop a bit later?
        
        if self.state == MovementState.SPIRAL:
            self.get_logger().info("GOT TAG, stopping spiral!")
            self.state = MovementState.FOLLOW
        self.drive_publisher.publish(self.velocity_result)
        self.print_debug()

    def turn(self):
        TURN_IN_PLACE_TIME: int = 7
        if self._spin_iter > TURN_IN_PLACE_TIME * self.SPIRAL_LOOP_RATE:
            self._turn_timer.destroy()
            self._spiral_iter = 0
            self.get_logger().info("SPIRALLING")
            self.current_curve_amt: float = 1.0
            self._spiral_timer = self.create_timer(1 / self.SPIRAL_LOOP_RATE, self.spiral)
            return
        if self.state != MovementState.SPIRAL:
            self.get_logger().info("HEREEEE")
            self.drive_publisher.publish(Float64MultiArray(data=[0.0, 0.0]))
            self._turn_timer.destroy()
            return

        from math import pi
        self.send_rotate_msg(2 * pi / TURN_IN_PLACE_TIME)
        self._spin_iter += 1
    
    def spiral(self):
        # Send messages to rotate for a full second

        # Keep spiraling forever, keep track of current iteration to make spiral larger
        SPIRAL_TIME = 20 # seconds
        CURVE_RATE: float = 1 / (MinimalSubscriber.SPIRAL_LOOP_RATE * SPIRAL_TIME)
    
        if self._spiral_iter > SPIRAL_TIME * self.SPIRAL_LOOP_RATE:
            self._spiral_timer.destroy()
            self.current_curve_amt = 1.0
            return
        if self.state != MovementState.SPIRAL:
            self.drive_publisher.publish(Float64MultiArray(data=[0.0, 0.0]))
            self.current_curve_amt
            self._spiral_timer.destroy()
            self.current_curve_amt = 1.0
            return

        self.send_spiral_msg(self.current_curve_amt)
        self.current_curve_amt -= CURVE_RATE

        self._spiral_iter += 1
        

    def send_rotate_msg(self, speed: float):
        """This method sends a Twist message to rotate the rover in-place,
        dependent on the specified speed.

        Args:
            speed (float): Angular speed to rotate at, in rad/s.
        """
        print(f"Sending rotate message with speed {speed}")
        self.velocity_result.data = [-1.0, 1.0]

        self.drive_publisher.publish(self.velocity_result)
    
    def send_spiral_msg(self, curve_amt: float):
        """This method sends a Twist message to drive the rover at a unit vector forward and
        the specified `rotation_speed`. Rotation speed should be decreased over time as to
        create a spiral pattern.

        Args:
            curve_amt (float): How much to curve 0.0-1.0.
        """
        # print(f"Sending spiral message with linear speed {linear_speed} and rotation speed {rotation_speed}")
        self.velocity_result.data = [1.0 - 2.0 * curve_amt, 1.0 ]

        self.drive_publisher.publish(self.velocity_result)
    
    def set_active(self, msg):
        if msg.data == False:
            self.state = MovementState.NONE
            self.drive_publisher.publish(Float64MultiArray(data=[0.0, 0.0]))
        else:
            self.get_logger().info("SPINNING!")

            self.state = MovementState.SPIRAL
            self._spin_iter = 0
            self._turn_timer = self.create_timer(1 / self.SPIRAL_LOOP_RATE, self.turn)

            


    def print_debug(self):
        if len(self.velocity_result.data) > 0:
            self.get_logger().info(
                f'moving autonomously with velocities ({self.velocity_result.data [0]}, {self.velocity_result.data [1]})'
            )


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
