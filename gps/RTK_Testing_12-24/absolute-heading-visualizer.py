# Single static base station, two rover GNSS module setup.
# Vector created between rover GNSS modules to get heading.

# Using a surveyed GNSS module as a base station,
# and two other GNSS modules as points on the rover receiving 
# RTCM3 correction data from the base station,
# this script creates a visualization of the rover's absolute heading.
# The visualizer acts as a unit circle, with North being 0 degrees facing right.

from serial import Serial
from ublox_gps import UbloxGps
from pyrtcm import RTCMReader
import cv2
import numpy as np
import math

base_serial_stream = Serial('/dev/ttyACM0', 460800, timeout=3)

rover_serial_stream_0 = Serial('/dev/ttyACM1', 460800, timeout=3)
rover_serial_stream_1 = Serial('/dev/ttyACM2', 460800, timeout=3)

print('Connected to serial ports.')

rtcm_reader = RTCMReader(base_serial_stream)

rover_gps_0 = UbloxGps(rover_serial_stream_0)
rover_gps_1 = UbloxGps(rover_serial_stream_1)

cv2.namedWindow("preview")

try:

    while True:

        (raw_data, parsed_data) = rtcm_reader.read()

        if raw_data is not None:
            rover_serial_stream_0.write(raw_data)
            rover_serial_stream_1.write(raw_data)
        else:
            print('No RTCM data received.')
        
        rover_coords_0 = rover_gps_0.geo_coords()
        rover_coords_1 = rover_gps_1.geo_coords()

        if rover_coords_0 is not None:
            print(f'GPS 0 Fix Type: {rover_coords_0.fixType}')
        
        if rover_coords_1 is not None:
            print(f'GPS 1 Fix Type: {rover_coords_1.fixType}')
        
        output_string = ''

        gps_0_coords = None
        gps_1_coords = None

        if rover_coords_0 is not None and rover_coords_0.flags.gnssFixOK == 1:
            output_string += f'Rover GPS 0: {rover_coords_0.lat:.7f}, {rover_coords_0.lon:.7f}'
            gps_0_coords = (rover_coords_0.lat, rover_coords_0.lon)
        else:
            output_string += 'Rover GPS 0: No fix'

        output_string += ' | '

        if rover_coords_1 is not None and rover_coords_1.flags.gnssFixOK == 1:
            output_string += f'Rover GPS 1: {rover_coords_1.lat:.7f}, {rover_coords_1.lon:.7f}'
            gps_1_coords = (rover_coords_1.lat, rover_coords_1.lon)
        else:
            output_string += 'Rover GPS 1: No fix'

        print(output_string)

        if gps_0_coords is None or gps_1_coords is None:
            continue

        # Visualize absolute heading vector

        vector = [gps_1_coords[0] - gps_0_coords[0], gps_1_coords[1] - gps_0_coords[1]]
        angle = math.degrees(math.atan2(vector[1], vector[0]))

        # convert to 0 to 360
        if angle < 0:
            angle += 360

        print(f'Angle: {angle}')

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

except KeyboardInterrupt:
    print('\nExiting...')
