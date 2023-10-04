# Description: This script is used to get the average position of the reference point.

import time
# import board
import busio
import adafruit_gps

# RX = board.RX
# TX = board.TX

# uart = busio.UART(TX, RX, baudrate=9600, timeout=30)

# for a computer, use the pyserial library for uart access
import serial
uart = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=10)

gps = adafruit_gps.GPS(uart, debug=False)

gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')

gps.send_command(b'PMTK220,1000')

last_print = time.monotonic()

num_count = 50
pos_list = []

while True:

    gps.update()

    current = time.monotonic()
    if current - last_print >= 1.0:
        last_print = current
        if not gps.has_fix:
            print('Waiting for fix...')
            continue
        print('=' * 40)  # Print a separator line.
        print('Latitude: {0:.6f} degrees'.format(gps.latitude))
        print('Longitude: {0:.6f} degrees'.format(gps.longitude))

        pos_list.append([gps.latitude, gps.longitude])
        num_count -= 1

        if num_count == 0:
            break


pos_avg = [0, 0]

for pos in pos_list:
    pos_avg[0] += pos[0]
    pos_avg[1] += pos[1]

pos_avg[0] /= len(pos_list)
pos_avg[1] /= len(pos_list)

print("Average position: ", pos_avg)