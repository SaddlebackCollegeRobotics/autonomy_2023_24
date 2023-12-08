from ublox_gps import UbloxGps
import serial


class GPS_Model:

    def __init__(self, dev_path: str):
        try:
            self.gps = UbloxGps(serial.Serial(dev_path, baudrate=38400, timeout=1))
        except Exception:
            print("Error: Could not open serial port.")
            exit(1)

    def get_coords(self):
        try:
            coords = self.gps.geo_coords()
            return coords.lat, coords.lon
        except (ValueError, IOError):
            print("Error: Could not parse GPS coords.")
        return None
