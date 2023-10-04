# Ros2 imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

# General imports
import adafruit_gps
import serial
import os
import subprocess
import sys


class GPS_Driver(Node):

    # Initialize publisher
    def __init__(self):

        # Give the node a name.
        super().__init__('gps_driver')

        # Initialize and connect to GPS
        self.init_GPS()

        # Specify data type and topic name. Specify queue size (limit amount of queued messages)
        self.publisher_ = self.create_publisher(NavSatFix, 'gps_position', 10)

        self.msg = NavSatFix()

        # Create a timer that will call the 'timer_callback' function every timer_period second.
        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    # Initialize and connect to GPS
    def init_GPS(self):
        
        devpath = self.get_devpath()

        if devpath is None:
            self.get_logger().info("No GPS device found")
            sys.exit()

        uart = serial.Serial(devpath, baudrate=9600, timeout=10)

        self.gps = adafruit_gps.GPS(uart, debug=False)
        self.gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
        self.gps.send_command(b'PMTK220,1000')

    
    # Get path to GPS device
    def get_devpath(self):

        serial_number = "e688dad60ba4ec11b308e589a29c855c"
        
        getter_script = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../../../share/gps_driver/find_devpath.bash')
        device_list = subprocess.run(["\"" + getter_script + "\""], stdout=subprocess.PIPE, text=True, shell=True, executable='/bin/bash').stdout.splitlines()
        
        for device in device_list:
            splitStr = device.split(" - ")

            if serial_number in splitStr[1]:
                return splitStr[0]
        
        return None


    def timer_callback(self):

        self.gps.update()

        # Wait for satellite connection
        if not self.gps.has_fix:
            self.get_logger().info("Waiting for fix...")
            return

        self.msg.latitude = self.gps.latitude
        self.msg.longitude = self.gps.longitude

        # Publish the message.
        self.publisher_.publish(self.msg)

        # Log the message.
        location = "Publishing: " + str(self.msg.latitude) + " " + str(self.msg.longitude)
        self.get_logger().info(location)



def main(args=None):
    rclpy.init(args=args)

    gps_driver = GPS_Driver()

    rclpy.spin(gps_driver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gps_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
