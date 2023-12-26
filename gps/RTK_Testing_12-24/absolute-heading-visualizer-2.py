# Single static base station GNSS module.
# Moving base station, and moving rover GNSS modules are on the same vehicle.
# Vector created between onboard the moving rover module to get heading.
# RTCM3 correction data is sent from the static base module to the moving base module.
# This give the moving base module an absolute position.
# The moving base module sends RTCM3 correction data to the moving rover module.
# This gives the moving rover module a relative position to the moving base module.
# A vector can then be created to have very accurate heading information.

from serial import Serial
from ublox_gps import UbloxGps
from pyrtcm import RTCMReader
import cv2
import numpy as np
import math
from threading import Thread

def cool_thread(moving_base_serial_stream, moving_rover_serial_stream):

    mb_gps = UbloxGps(moving_base_serial_stream)
    mr_gps = UbloxGps(moving_rover_serial_stream)

    while True:
        mb_coords = mb_gps.geo_coords()
        mr_coords = mr_gps.geo_coords()

        if mb_coords is not None:
            print(f'Moving base GPS: {mb_coords.lat:.7f}, {mb_coords.lon:.7f}')
        else:
            print('Moving base GPS: No fix')

        if mr_coords is not None:
            print(f'Moving rover GPS: {mr_coords.lat:.7f}, {mr_coords.lon:.7f}')
        else:
            print('Moving rover GPS: No fix')


static_base_serial_stream = Serial('/dev/ttyACM0', 460800, timeout=3)

moving_base_serial_stream = Serial('/dev/ttyACM1', 460800, timeout=3)
moving_rover_serial_stream = Serial('/dev/ttyACM2', 460800, timeout=3)

print('Connected to serial ports.')

sb_rtcm_reader = RTCMReader(static_base_serial_stream)
mb_rtcm_reader = RTCMReader(moving_base_serial_stream)

thread = Thread(target=cool_thread, args=(moving_base_serial_stream, moving_rover_serial_stream))
thread.start()

cv2.namedWindow("preview")

try:

    while True:

        byte_buffer = static_base_serial_stream.read_all()
        RTCMReader.pa
        


        (sb_raw_rtcm, parsed_data_0) = sb_rtcm_reader.read()
        (mb_raw_rtcm, parsed_data_1) = mb_rtcm_reader.read()

        if sb_raw_rtcm is not None:
            moving_base_serial_stream.write(sb_raw_rtcm)
        else:
            print('No RTCM data received from static base.')

        if mb_raw_rtcm is not None:
            moving_rover_serial_stream.write(mb_raw_rtcm)
        else:
            print('No RTCM data received from moving base.')
        
        continue
        
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
