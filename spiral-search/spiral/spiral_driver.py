import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3

from time import sleep

from itertools import count



class SpiralDriver(Node):

    # Frequency at which this node will publish messages, in Hz
    LOOP_RATE: float = 15

    # Speed to rotate at, rad/s
    # This is for IN-PLACE rotation
    ROTATION_SPEED: float = 0.5

    def __init__(self):
        super().__init__('spiral_driver')

        print("Spiral driver node started.")

        self._publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self._twist_msg = Twist()

        loop_rate = self.create_rate(SpiralDriver.LOOP_RATE, self.get_clock())

        # Send messages to rotate for a full second
        TURN_IN_PLACE_TIME: int = 10
        from math import pi
        for _ in range(TURN_IN_PLACE_TIME * SpiralDriver.LOOP_RATE):
            self.send_rotate_msg(2 * pi / TURN_IN_PLACE_TIME)
            sleep(1/SpiralDriver.LOOP_RATE)

        # Keep spiraling forever, keep track of current iteration to make spiral larger
        #TODO: Change this sometime
        INITIAL_ROT_SPEED: float = 3.0
        SPIRAL_RATE: float = 499/500 # Constant to multiply rotation speed by on each iteration
        current_rot_speed: float = INITIAL_ROT_SPEED
        while True:
            self.send_spiral_msg(current_rot_speed)
            # loop_rate.sleep()
            sleep(1/SpiralDriver.LOOP_RATE)
            current_rot_speed *= SPIRAL_RATE

    def send_rotate_msg(self, speed: float):
        """This method sends a Twist message to rotate the rover in-place,
        dependent on the specified speed.

        Args:
            speed (float): Angular speed to rotate at, in rad/s.
        """
        print(f"Sending rotate message with speed {speed}")
        self._twist_msg.linear = Vector3(x=0.0, y=0.0, z=0.0)
        self._twist_msg.angular = Vector3(x=0.0, y=0.0, z=speed)

        self._publisher.publish(self._twist_msg)
    
    def send_spiral_msg(self, rotation_speed: float, linear_speed: float = 20.0):
        """This method sends a Twist message to drive the rover at a unit vector forward and
        the specified `rotation_speed`. Rotation speed should be decreased over time as to
        create a spiral pattern.

        Args:
            rotation_speed (float): How fast the rover should rotate in rad/s.
        """
        print(f"Sending spiral message with linear speed {linear_speed} and rotation speed {rotation_speed}")
        self._twist_msg.linear = Vector3(x=linear_speed, y=0.0, z=0.0)
        self._twist_msg.angular = Vector3(x=0.0, y=0.0, z=rotation_speed)

        self._publisher.publish(self._twist_msg)




def main(args=None):
    rclpy.init(args=args)

    spiral_driver = SpiralDriver()

    rclpy.spin(spiral_driver)

    rclpy.shutdown()


if __name__ == '__main__':
    main()