"""Simple tui program to simulate gps heading for testing purposes.
Controls are:
    'a' => Rotate left
    'd' => Rotate right
    'q' => Quit
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from ublox_ubx_msgs.msg import UBXNavRelPosNED
from rclpy.qos import qos_profile_sensor_data

from threading import Thread
import curses


class GpsInteractive(Node):
    def __init__(self):
        super().__init__("heading_repub")

        self._gps_pub = self.create_publisher(
            Float64, "/gps/moving_rover/heading_angle", qos_profile_sensor_data
        )

        self._current_angle = 0.0

    def changeAngle(self, delta: float):
        self._current_angle += delta
        self._current_angle = float(self._current_angle % 360)

        self._gps_pub.publish(Float64(data=self._current_angle))


def input_handler(stdscr: curses.window, node: GpsInteractive):
    ANGLE_DELTA = 2

    # Init ncurses
    curses.cbreak()  # Process input without <Enter>
    curses.noecho()  # Don't echo user input
    stdscr.keypad(True)  # Custom curses key-codes

    try:
        while key := stdscr.getkey():
            if key == "a":
                stdscr.clear()
                node.changeAngle(-ANGLE_DELTA)
                stdscr.addstr("LEFT")
            elif key == "d":
                stdscr.clear()
                node.changeAngle(ANGLE_DELTA)
                stdscr.addstr("RIGHT")
            elif key == "q":
                print("Safely exiting...")
                break
    except KeyboardInterrupt:
        print("Interrupt received. Safely exiting...")


def main():
    rclpy.init()

    try:
        node = GpsInteractive()

        # Input handling on a seperate thread
        input_thread = Thread(target=curses.wrapper, args=(input_handler, node))
        input_thread.start()

        rclpy.spin(node)

    except KeyboardInterrupt:
        print("Interrupt received. Safely exiting...")

    input_thread.join()
    rclpy.shutdown()
