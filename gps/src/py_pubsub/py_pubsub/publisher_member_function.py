import rclpy
from rclpy.node import Node
from serial import Serial, SerialException
from crc import Calculator, Crc32
# pip3 install crc
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped
from rtcm_msgs.msg import Message as RTCMMessage

import numpy as np


class MinimalPublisher(Node):

    def __init__(self):

        # Give the node a name.
        super().__init__('minimal_publisher')

        self.navsatfix_publisher = self.create_publisher(NavSatFix, '/gps/moving_rover/navsatfix', qos_profile=qos_profile_sensor_data)
        self.heading_publisher = self.create_publisher(PoseWithCovarianceStamped, '/gps/moving_rover/heading', qos_profile=qos_profile_sensor_data)

        self.subscription = self.create_subscription(
            RTCMMessage,
            '/gps/static_base/rtcm_correction',
            self.rtcm_callback,
            10)
        # self.subscription  # prevent unused variable warning

        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.delimiter = ','

        self.dev_path = "/dev/ttyACM1"

        try:
            self.serial = Serial(self.dev_path, baudrate=460800)
        except SerialException:
            print("Error: Could not find device on:", self.dev_path)
            exit(1)

        self.serial.flush()
        # self.serial.reset_input_buffer() # Do we need this?

        self.crc_calculator = Calculator(Crc32.CRC32, optimized=True)

        self.navsatfix_msg = NavSatFix()
        self.heading_msg = PoseWithCovarianceStamped()

        self.carr_soln_dict = {"0":"None", "1":"Float", "2":"Fixed"}

        # TODO - handle kb interrupt and sigkill


    # Author: AutomaticAddison.com
    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.
        
        Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.
        
        Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    
        return [qx, qy, qz, qw]
    
    def rtcm_callback(self, msg: RTCMMessage):
        
        # Take incoming rtcm correction data, create a checksum,
        # and send data along with checksum to microcontroller.

        # TODO - Add checksum.

        data_buffer = msg.message.tobytes()
        self.serial.write(data_buffer)

    def timer_callback(self):

        line = self.serial.readline().decode().rstrip()

        print("="*40)

        if len(line) == 0:
            print("Warning: Data length zero.")
            return
        
        try:
            last_delim_index = line.rindex(',')
        except ValueError:
            print("Warning: No delimiter, skipping data.")
            return

        expected_crc = line[last_delim_index + 1 :]
        data = line[:last_delim_index + 1]

        if expected_crc != str(self.crc_calculator.checksum(data.encode())):
            print("Warning: Checksum failed!")
            return
        
        data_array = data[:-1].split(self.delimiter)

        gnss_fix_ok, rel_pos_valid, mb_carr_soln_type, mr_carr_soln_type, latitude, longitude, relative_heading = data_array

        print("Moving Base Carr Soln: ", end="")
        print(self.carr_soln_dict[mb_carr_soln_type])

        print("Moving Rover Carr Soln: ", end="")
        print(self.carr_soln_dict[mr_carr_soln_type])

        time_stamp = self.get_clock().now().to_msg()

        # NavSatFix Message
        # https://docs.ros2.org/latest/api/sensor_msgs/msg/NavSatFix.html

        if int(gnss_fix_ok) == 0:
            print("Warning: No GPS fix!")
            return

        self.navsatfix_msg.header.stamp = time_stamp
        self.navsatfix_msg.header.frame_id = "map"

        self.navsatfix_msg.status.status = int(gnss_fix_ok) - 1 # -1 = No fix for msg type
        
        self.navsatfix_msg.latitude = float(latitude) / 10000000
        self.navsatfix_msg.longitude = float(longitude) / 10000000

        # TODO - Add covariance matrix
        self.navsatfix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.navsatfix_publisher.publish(self.navsatfix_msg)

        # Heading Message
        # https://docs.ros.org/en/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html

        if int(rel_pos_valid) == 0:
            print("Warning: Relative position is invalid!")
            return

        yaw = np.radians(float(relative_heading) / 100000)

        qx, qy, qz, qw = self.get_quaternion_from_euler(0, 0, yaw)

        self.heading_msg.header.stamp = time_stamp
        self.heading_msg.header.frame_id = "map"

        # TODO - Add covariance matrix

        self.heading_msg.pose.pose.orientation.x = qx
        self.heading_msg.pose.pose.orientation.y = qy
        self.heading_msg.pose.pose.orientation.z = qz
        self.heading_msg.pose.pose.orientation.w = qw

        self.heading_publisher.publish(self.heading_msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
