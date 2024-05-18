from serial import Serial
from pyrtcm import RTCMReader

stream = Serial('/dev/ttyACM1', 460800, timeout=3)
rtr = RTCMReader(stream)

try:
    while True:
        (raw_data, parsed_data) = rtr.read()
        print(raw_data)
except KeyboardInterrupt:
    print('\nExiting...')
