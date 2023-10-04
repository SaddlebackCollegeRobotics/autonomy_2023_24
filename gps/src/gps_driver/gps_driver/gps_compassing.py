# Using 2 GPS, get vector between them, and draw it on screen

import adafruit_gps
import serial
import time
import math
import numpy as np
import cv2

class GPS_Model:

    def __init__(self, dev_path: str):
        self.dev_path = dev_path

    def setup(self) -> bool:
    
        try:
            self.gps = adafruit_gps.GPS(serial.Serial(self.dev_path, baudrate=9600, timeout=10), debug=False)
            self.gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
            self.gps.send_command(b'PMTK220,1000')

            print("GPS successfully setup")
        except:
            print("GPS not found")
            return False
        
        return True

    def update(self):
        
        self.gps.update()
        self.latitude = self.gps.latitude
        self.longitude = self.gps.longitude


    def has_fix(self) -> bool:
        return self.gps.has_fix
    

ref_pos = [33.553255480000004, -117.66242384000005]

def differential_gnss(gps_list: list):

    error_vec = [ref_pos[0] - gps_list[0].latitude, ref_pos[1] - gps_list[0].longitude]

    fixed_pos1 = ref_pos
    # fixed_pos1 = [ref_pos[0] + error_vec[0], ref_pos[1] + error_vec[1]]
    fixed_pos2 = [gps_list[1].latitude + error_vec[0], gps_list[1].longitude + error_vec[1]]
    # fixed_pos2 = [gps_list[2].latitude + error_vec[0], gps_list[2].longitude + error_vec[1]]

    return [fixed_pos1, fixed_pos2]


def normal_gnss(gps_list: list):

    return [gps_list[0].latitude, gps_list[0].longitude], [gps_list[1].latitude, gps_list[1].longitude]


cv2.namedWindow("preview")
gps_list = [GPS_Model("/dev/ttyUSB0"), GPS_Model("/dev/ttyUSB1")]

for gps in gps_list:
    success = gps.setup()
    if not success:
        print("GPS setup failed")
        exit()

last_print = time.monotonic()

while True:

    has_fix = True

    for gps in gps_list:

        gps.update()

        if not gps.has_fix():
            print('Waiting for fix...')
            has_fix = False

    if not has_fix:
        continue

    current = time.monotonic()

    if current - last_print >= 1.0:
        last_print = current
        
        # (pos1, pos2) = differential_gnss(gps_list)
        (pos1, pos2) = normal_gnss(gps_list)

        vector = [pos2[0] - pos1[0], pos2[1] - pos1[1]]
        # magnitude = math.sqrt(vector[0] ** 2 + vector[1] ** 2)

        # unit_vector = [vector[0] / magnitude, vector[1] / magnitude]

        # angle = math.degrees(math.atan2(unit_vector[1], unit_vector[0]))
        angle = math.degrees(math.atan2(vector[1], vector[0]))

        # convert to 0 to 360
        if angle < 0:
            angle += 360

        print('=' * 40)
        print(time.time())
        print("Angle: ", angle)
        print("base: ", pos1, " rover: ", pos2)

        frame = np.ones((512, 512, 3), np.uint8) * 255

        view_vector = [math.cos(math.radians(angle)), math.sin(math.radians(angle))]

        # draw line
        line_length = 200
        line_thickness = 7
        line_start = (256, 256)
        line_end = (256 + int(view_vector[0] * line_length), 256 + int(view_vector[1] * line_length))
        line_color = (0, 0, 0)        

        cv2.line(frame, line_start, line_end, line_color, line_thickness)

        cv2.imshow("preview", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

        # print("latitude1: ", gps_list[0].latitude, " longitude1: ", gps_list[0].longitude)
        # print("latitude:2 ", gps_list[1].latitude, " longitude:2 ", gps_list[1].longitude)