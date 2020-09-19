import spidev
import serial
from io import BytesIO
from time import sleep
from picamera import PiCamera
import os
import zlib

#camera = PiCamera()
#camera.resolution = (1024, 768)
#camera.rotation = 180


# Enable SPI
spi = spidev.SpiDev()

# Open a connection to a specific bus and device (chip select pin)
# spi.open(bus, CS_pin) bus = 0, CS_pin = 0 or 1
spi.open(0, 0)
ser = serial.Serial('/dev/ttyS0', 115200)  # open serial port

# Set SPI speed and mode
spi.max_speed_hz = 10000000
spi.mode = 0


# Camera capture
#camera.capture("my_image.jpg")
my_file = open("pluto128.jpg", "rb")


# PAYLOAD

my_file.seek(0, os.SEEK_END)
size = my_file.tell()						# Size calculator

#print("size: " + str(size))					# DEBUG
#print("parts: " + str(int(size/2048)) )				# DEBUG

my_file.seek(0)

ID = int(size/2048)

for i in range(ID):
	list_bytes = list(my_file.read(2048))			# Payload (image)
	list_bytes.insert(0, 0)					# Size L
	list_bytes.insert(0, 8)					# Size H
	list_bytes.insert(0,  (ID - i) & 0xFF)			# ID L  (part)
	list_bytes.insert(0, ((ID - i) & 0xFF00) >> 8)		# ID H

	crc32 = zlib.crc32( bytes(list_bytes) )			# CRC
	list_bytes.append( (crc32 & 0xFF000000) >> 24 )
	list_bytes.append( (crc32 & 0xFF0000) >> 16 )
	list_bytes.append( (crc32 & 0xFF00) >> 8 )
	list_bytes.append(  crc32 & 0xFF )

#	byt = ["0x%02x" % n for n in list_bytes]		# DEBUG
#	print(byt)						# DEBUG hex
#	print(list_bytes)					# DEBUG dec
#	print("crc: " + str(crc32))				# DEBUG

	spi.writebytes(list_bytes)				# SPI Write payload

else:
	remainder = size - my_file.tell()			# remainder payload
#	print("resto: " + str(remainder))			# DEBUG

	list_bytes = list( my_file.read(remainder) )
	list_bytes.insert(0, remainder & 0xFF)			# Size L
	list_bytes.insert(0, (remainder & 0xFF00) >> 8)		# Size H
	list_bytes.insert(0, 0)					# ID L (part)
	list_bytes.insert(0, 0)					# ID H

	crc32 = zlib.crc32( bytes(list_bytes) )			# CRC
	list_bytes.append( (crc32 & 0xFF000000) >> 24 )
	list_bytes.append( (crc32 & 0xFF0000) >> 16 )
	list_bytes.append( (crc32 & 0xFF00) >> 8 )
	list_bytes.append(  crc32 & 0xFF )

#	byt = ["0x%02x" % n for n in list_bytes]		# DEBUG
#	print(byt)						# DEBUG hex
#	print(list_bytes)					# DEBUG dec
#	print("crc: " + str(crc32))				# DEBUG

	spi.writebytes(list_bytes)				# SPI Write payload


ser.write(b'END')						# write a string


# close port
ser.close()
spi.close()
#camera.close()
my_file.close()
