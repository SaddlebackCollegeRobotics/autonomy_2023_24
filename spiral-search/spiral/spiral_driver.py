import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from time import sleep
from math import pi
import logging

CONTROL_INPUT_TOPIC = '/drive/control_input'
MESSAGE_TYPE_FLOAT64_MULTI_ARRAY = Float64MultiArray
logger = logging.getLogger('spiral_driver')

# SpiralDriver is inheriting from Node class
class SpiralDriver(Node):

    # Constructor
    def __init__(self):
        
        # Frequency (Hz) at which a node will publish messages
        self.loop_rate: float = 15 
        # Speed to rotate at (rad/s), used for IN-PLACE rotation
        self.rotation_speed: float = 0.5
        # Send messages to rotate for a full second
        self.turn_in_place_time: int = 7
        self.spiral_curvature_rate: float = 2499/2500
        self.current_spiral_curvature: float = 99/100
        
        super().__init__('spiral_driver')

        print("Spiral driver node started.\n")

        try:
            self._publisher = self.create_publisher(MESSAGE_TYPE_FLOAT64_MULTI_ARRAY, CONTROL_INPUT_TOPIC, 10)
        except Exception as e:
            logger.error(f"Failed to create publisher: {e}")
            return

        self._publisher = self.create_publisher(MESSAGE_TYPE_FLOAT64_MULTI_ARRAY,CONTROL_INPUT_TOPIC, 10)

        self._vel_msg = Float64MultiArray()

        self.initialize_clear_area()

        self.initialize_rotation()

        # Keep spiraling forever, keep track of current iteration to make spiral larger
        #TODO: Change this sometime
        while True:
            self.send_spiral_msg(self.current_spiral_curvature)
            sleep(1/SpiralDriver.loop_rate)
            self.current_spiral_curvature *= self.spiral_curvature_rate

    def initialize_clear_area(self):
        for _ in range(20):
            self._vel_msg.data = [0.0, 0.0]
            self._publisher.publish(self._vel_msg)
            sleep(0.5)

    def initialize_rotation(self):
        for _ in range(self.turn_in_place_time * SpiralDriver.loop_rate):
            self.send_rotate_msg(2 * pi / self.turn_in_place_time)
        sleep(1/SpiralDriver.loop_rate)
    
    def send_rotate_msg(self, speed: float):
        """This method sends a Twist message to rotate the rover in-place,
        dependent on the specified speed.

        Args:
            speed (float): Angular speed to rotate at, in rad/s.
        """
        print(f"Sending rotate message with speed {speed}")
        self._vel_msg.data = [-1.0, 1.0]

        self._publisher.publish(self._vel_msg)
    
    def send_spiral_msg(self, curve_amt: float):
        """This method sends a Twist message to drive the rover at a unit vector forward and
        the specified `rotation_speed`. Rotation speed should be decreased over time as to
        create a spiral pattern.

        Args:
            rotation_speed (float): How fast the rover should rotate in rad/s.
        """
        # print(f"Sending spiral message with linear speed {linear_speed} and rotation speed {rotation_speed}")
        self._vel_msg.data = [1.0 - 1.0 * curve_amt, 1.0 ]

        self._publisher.publish(self._vel_msg)


def main(args=None):

    rclpy.init(args=args)

    spiral_driver = SpiralDriver()

    rclpy.spin(spiral_driver)

    rclpy.shutdown()


if __name__ == '__main__':
    main()