# Using 2 GPS, get vector between them, and draw it on screen

from gps_driver.gps_driver.ublox_gps_driver import GPS_Model
import serial
import time
import math
import numpy as np
import cv2


def get_coords(gps_list):

    coords_list = []

    for gps in gps_list:

        coords = gps.get_coords()

        if coords is not None:
            coords_list.append(coords)

    if len(coords_list) == 2:
        return coords_list[0], coords_list[1]
    else:
        return None, None
      

cv2.namedWindow("preview")

gps_list = [GPS_Model("/dev/ttyACM1"), GPS_Model("/dev/ttyACM2")]

while True:
        
    (pos1, pos2) = get_coords(gps_list)

    if pos1 is None or pos2 is None:
        continue

    vector = [pos2[0] - pos1[0], pos2[1] - pos1[1]]
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