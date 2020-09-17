import spidev
import serial
from time import sleep
from picamera import PiCamera

camera = PiCamera()

# We only have SPI bus 0 available to us on the Pi
bus = 0

#Device is the chip select pin. Set to 0 or 1, depending on the connections
CS_pin = 0

# Enable SPI
spi = spidev.SpiDev()

# Open a connection to a specific bus and device (chip select pin)
spi.open(bus, CS_pin)
ser = serial.Serial('/dev/ttyS0', 115200)  # open serial port

# Set SPI speed and mode
spi.max_speed_hz = 10000000
spi.mode = 0

# Message
msg = [0x76]

spi.writebytes2(b'hello')
ser.write(b'hello')     # write a string

# close port
ser.close()
spi.close()
