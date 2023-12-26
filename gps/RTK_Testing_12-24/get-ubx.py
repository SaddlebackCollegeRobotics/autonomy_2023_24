from pyubx2 import UBXReader, GET, UBX_PROTOCOL, UBXMessage
from serial import Serial

serial_stream = Serial('/dev/ttyACM0', 460800, timeout=3)

reader = UBXReader(serial_stream, GET, protfilter=UBX_PROTOCOL)

while True:

    data = reader.read()
    UBXMessage(,)
    