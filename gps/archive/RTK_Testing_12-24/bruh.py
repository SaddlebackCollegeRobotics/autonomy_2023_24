from serial import Serial
from ublox_gps import UbloxGps
from pyrtcm import RTCMReader

static_base_serial_stream = Serial('/dev/ttyACM0', 460800, timeout=3)

moving_base_serial_stream = Serial('/dev/ttyACM1', 460800, timeout=3)
moving_rover_serial_stream = Serial('/dev/ttyACM2', 460800, timeout=3)

print('Connected to serial ports.')

sb_rtcm_reader = RTCMReader(static_base_serial_stream)
mb_rtcm_reader = RTCMReader(moving_base_serial_stream)

mb_gps = UbloxGps(moving_base_serial_stream)
mr_gps = UbloxGps(moving_rover_serial_stream)

try:

    while True:

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
        
except KeyboardInterrupt:
    print('\nExiting.')
    static_base_serial_stream.close()
    moving_base_serial_stream.close()
    moving_rover_serial_stream.close()    