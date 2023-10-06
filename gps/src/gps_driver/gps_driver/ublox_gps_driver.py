from ublox_gps import UbloxGps
import serial


class GPSModel:

    def __init__(self, dev_path: str):
        self.gps = UbloxGps(serial.Serial("dev_path", baudrate=38400, timeout=1))

    def get_coords(self):
        try:
            coords = self.gps.geo_coords()
            return coords.lat, coords.lon
        except (ValueError, IOError):
            print("Error: Could not parse GPS coords.")
        return None


def main():
    gps = GPSModel('/dev/ttyACM0')

    while True:
        try:
            lat, lon = gps.get_coords()
            print("Rel Latitude: {:.7f}, Rel Longitude: {:.7f}".format(lat, lon))
        except KeyboardInterrupt:
            print("exit")

if __name__ == '__main__':
    main()
