from serial import Serial

print('Connecting to serial ports...')

base_serial_stream = Serial('/dev/ttyACM0', 9600, timeout=3)

rover_serial_stream_0 = Serial('/dev/ttyACM1', 9600, timeout=3)
# rover_serial_stream_1 = Serial('/dev/ttyACM2', 9600, timeout=3)

print('Connected to serial ports.')

try:

    while True:
        # byte_buffer = base_serial_stream.read(1200)

        rover_serial_stream_0.write(base_serial_stream.read(1200))
        # rover_serial_stream_1.write(byte_buffer)
except KeyboardInterrupt:
    print('\nExiting...')
