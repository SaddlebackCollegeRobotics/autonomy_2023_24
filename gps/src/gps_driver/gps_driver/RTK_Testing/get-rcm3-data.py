from serial import Serial
from pyrtcm import RTCMReader

stream = Serial('/dev/ttyACM0', 9600, timeout=3)
rtr = RTCMReader(stream)

try:
    while True:
        (raw_data, parsed_data) = rtr.read()
        print(parsed_data)
except KeyboardInterrupt:
    print('\nExiting...')
