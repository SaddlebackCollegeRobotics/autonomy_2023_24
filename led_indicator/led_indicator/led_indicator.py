import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from serial import Serial, SerialException


class LedIndicator(Node):

    def __init__(self):

        # Give the node a name.
        super().__init__("led_indicator")

        try:
            # TODO: Find the correct port
            self.serial = Serial("/dev/ttyUSB0", 9600)
        except SerialException:
            print("Error: Could not open serial port!")
            exit(1)

        self.subscription = self.create_subscription(
            String, "/led_pub", self.listener_callback, 10
        )

    def listener_callback(self, msg):
        serial_input = "#" + str(msg.data) + "%"
        self.serial.write(serial_input.encode())


def main(args=None):
    rclpy.init(args=args)

    led_indicator = LedIndicator()

    rclpy.spin(led_indicator)

    led_indicator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
