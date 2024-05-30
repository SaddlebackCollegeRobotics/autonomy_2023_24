from serial import Serial
from pyrtcm import RTCMReader

stream = Serial('/dev/ttyACM0', 460800, timeout=3)
stream = Serial('/dev/ttyACM1', 460800, timeout=3)
rtr_0 = RTCMReader(stream)
rtr_1 = RTCMReader(stream)

try:
    while True:
        (raw_data_0, parsed_data_0) = rtr_0.read()
        (raw_data_1, parsed_data_1) = rtr_1.read()

        if raw_data_0 is not None:
            print(raw_data_0)
        if raw_data_1 is not None:
            print(raw_data_1)
        
except KeyboardInterrupt:
    print('\nExiting...')
