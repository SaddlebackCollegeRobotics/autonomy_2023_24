# Transfer RTCM3 correction data from a base station to a rover.

from serial import Serial
from ublox_gps import UbloxGps
from pyrtcm import RTCMReader

base_serial_stream = Serial('/dev/ttyACM0', 9600, timeout=3)
rover_serial_stream = Serial('/dev/ttyACM1', 9600, timeout=3)

print('Connected to serial ports.')

rtcm_reader = RTCMReader(base_serial_stream)

rover_gps = UbloxGps(rover_serial_stream)

try:

    while True:

        (raw_data, parsed_data) = rtcm_reader.read()

        if raw_data is not None:
            rover_serial_stream.write(raw_data)
        else:
            print('No RTCM data received.')
        
        rover_coords = rover_gps.geo_coords()
        
        if rover_coords is not None and rover_coords.flags.gnssFixOK == 1:
            print(f'Rover GPS: {rover_coords.lat:.7f}, {rover_coords.lon:.7f}')
        else:
            print('Rover GPS: No fix')

except KeyboardInterrupt:
    print('\nExiting...')
