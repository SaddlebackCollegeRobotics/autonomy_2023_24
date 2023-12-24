# Transfer RTCM3 correction data from a base station
# to two rover GNSS modules.

from serial import Serial
from ublox_gps import UbloxGps
from pyrtcm import RTCMReader

base_serial_stream = Serial('/dev/ttyACM0', 9600, timeout=3)

rover_serial_stream_0 = Serial('/dev/ttyACM1', 9600, timeout=3)
rover_serial_stream_1 = Serial('/dev/ttyACM2', 9600, timeout=3)

print('Connected to serial ports.')

rtcm_reader = RTCMReader(base_serial_stream)

rover_gps_0 = UbloxGps(rover_serial_stream_0)
rover_gps_1 = UbloxGps(rover_serial_stream_1)

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
        
        output_string = ''

        if rover_coords_0 is not None and rover_coords_0.flags.gnssFixOK == 1:
            output_string += f'Rover GPS 0: {rover_coords_0.lat:.7f}, {rover_coords_0.lon:.7f}'
        else:
            output_string += 'Rover GPS 0: No fix'

        output_string += ' | '

        if rover_coords_1 is not None and rover_coords_1.flags.gnssFixOK == 1:
            output_string += f'Rover GPS 1: {rover_coords_1.lat:.7f}, {rover_coords_1.lon:.7f}'
        else:
            output_string += 'Rover GPS 1: No fix'

        print(output_string)

except KeyboardInterrupt:
    print('\nExiting...')
