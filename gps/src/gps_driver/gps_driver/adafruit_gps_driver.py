import adafruit_gps
import serial
import math


class GPS_Model:

    def __init__(self, dev_path: str):

        self.latitude = math.nan
        self.longitude = math.nan
        
        try:
            self.gps = adafruit_gps.GPS(serial.Serial(dev_path, baudrate=9600, timeout=10), debug=False)
            self.gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
            self.gps.send_command(b'PMTK220,1000')
        except:
            print("GPS not found")

    def update(self):
        
        self.gps.update()
        self.latitude = self.gps.latitude
        self.longitude = self.gps.longitude

    def has_fix(self) -> bool:
        return self.gps.has_fix