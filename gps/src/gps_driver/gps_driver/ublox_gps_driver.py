# Description: This is a driver for the ublox gps module. It is used to get the latitude and longitude of the gps module.

from ublox_gps import UbloxGps
import serial
# Can also use SPI here - import spidev
# I2C is not supported

port = serial.Serial('/dev/ttyACM0', baudrate=38400, timeout=1)
gps = UbloxGps(port)
relPointLat = 33.5530609
relPointLon = -117.6644828

        

def run():

    try: 
        print("Listenting for UBX Messages.")
       
        while True:
            try: 
                coords = gps.geo_coords()
                coordsRefLat = coords.lat - relPointLat
                coordsRefLon = coords.lon - relPointLon
                # Format latitude and longitude to 7 decimal places and print
                # print("Latitude: {:.7f}, Longitude: {:.7f}".format(coords.lat, coords.lon))
                print("Rel Latitude: {:.7f}, Rel Longitude: {:.7f}".format(coordsRefLat, coordsRefLon))


            except (ValueError, IOError) as err:
                print(err)

    except KeyboardInterrupt:
        print("\nExiting Program")

    finally:
        port.close()

if __name__ == '__main__':
  run()