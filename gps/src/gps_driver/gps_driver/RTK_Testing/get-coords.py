from ublox_gps_driver import GPS_Model
from time import sleep

gps = GPS_Model("/dev/ttyACM1")

while True:
    coords = gps.get_coords()
    if coords is not None:
        print(f'{coords[0]:.7f}, {coords[1]:.7f}')
    sleep(1)
