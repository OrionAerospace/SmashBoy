import spidev
import serial
from io import BytesIO
from time import sleep
from picamera import PiCamera
import os
import zlib


CONFIG 	= b'\x00'
FINISH 	= b'\xFF'
SUSPEND = b'\x55'
PICTURE = b'\x0F'
SEND 	= b'\xC3'
REPEAT 	= b'\xA5'	# if (status != REPEAT) -> NEXT

state = CONFIG

while 1:
# ----------------------------------------------------------------------------------------------------------------------
	if state == CONFIG:

		print("---=-=-= INITIAL CONFIGURATION =-=-=---")

		# camera = PiCamera()
		# camera.resolution = (1024, 768)
		# camera.rotation = 180

		# Enable SPI
		spi = spidev.SpiDev()

		# Open a connection to a specific bus and device (chip select pin)
		# spi.open(bus, CS_pin) bus = 0, CS_pin = 0 or 1
		spi.open(0, 0)
		uart = serial.Serial('/dev/ttyS0', 115200)  # open serial port

		# Set SPI speed and mode
		spi.max_speed_hz = 1000000
		spi.mode = 0

		state = SUSPEND
# ----------------------------------------------------------------------------------------------------------------------
	elif state == FINISH:
		# close port
		uart.close()
		spi.close()
		# camera.close()
		my_file.close()

		print("THE END")					# DEBUG
# ----------------------------------------------------------------------------------------------------------------------
	elif state == SUSPEND:
		print("---=-=-=  ATIVIDADE SUSPENSA  =-=-=---")
		state = uart.read(1)
		uart.write(state)
# ----------------------------------------------------------------------------------------------------------------------
	elif state == PICTURE:
		print("---=-=-= TAKE PICTURE =-=-=---")
		# Camera capture
		# camera.capture("my_image.jpg")

		state = uart.read(1)
		uart.write(state)
# ----------------------------------------------------------------------------------------------------------------------
	elif state == SEND:
		print("---=-=-= SEND PICTURE =-=-=---")

		my_file = open("pluto128.jpg", "rb")

		my_file.seek(0, os.SEEK_END)
		size = my_file.tell()  					# File size calculator
		my_file.seek(0)

		print("size: " + str(size))  				# DEBUG
		# print("parts: " + str(int(size/2024)) )		# DEBUG

		size_bytes = (size).to_bytes(4, byteorder='big')  	# int to byte
		list_size_bytes = list(size_bytes)  			# to list

		loraPyldArray = 253 * 16  				# 253 Bytes = Payload LoRa, 253*16 = 4048 Bytes Payload Buffer STM32
		ID = int( size / loraPyldArray )

		for i in range(ID):
			list_bytes = list(my_file.read(loraPyldArray))		# Payload (image)
			list_bytes.insert(0, loraPyldArray & 0xFF)		# Size L
			list_bytes.insert(0, (loraPyldArray >> 8) & 0xFF)	# Size H
			list_bytes.insert(0,  (ID - i) & 0xFF)			# ID L  (part)
			list_bytes.insert(0, ((ID - i) & 0xFF00) >> 8)		# ID H
			
			for j in range(4):
				list_bytes.append(list_size_bytes[j])		# size

			crc32 = zlib.crc32( bytes(list_bytes) )			# CRC32
			list_bytes.append( (crc32 & 0xFF000000) >> 24 )
			list_bytes.append( (crc32 & 0xFF0000) >> 16 )
			list_bytes.append( (crc32 & 0xFF00) >> 8 )
			list_bytes.append(  crc32 & 0xFF )

			byt = ["0x%02x" % n for n in list_bytes]		# DEBUG
			print(byt)						# DEBUG hex
		#	print(list_bytes)					# DEBUG dec
			print("crc: " + str(crc32))				# DEBUG

			status = REPEAT
			while(REPEAT == status):
				spi.writebytes(list_bytes)  			# SPI Write payload
				status = uart.read(1)
				uart.write(status)

			if(status == SUSPEND):
				state = SUSPEND
				break						# Sai do loop for se for solicitado reset

			else:
				remainder = size - my_file.tell()  		# remainder payload
				print("resto: " + str(remainder))  		# DEBUG

				list_bytes = list(my_file.read(remainder))
				list_bytes.insert(0, remainder & 0xFF)  		# Size L
				list_bytes.insert(0, (remainder & 0xFF00) >> 8)  	# Size H
				list_bytes.insert(0, 0)  				# ID L (part)
				list_bytes.insert(0, 0)  				# ID H

				while remainder < loraPyldMatrix:
					list_bytes.append(0)
					remainder = remainder + 1

				for j in range(4):
					list_bytes.append(list_size_bytes[j])	# size

				crc32 = zlib.crc32(bytes(list_bytes))  		# CRC32
				list_bytes.append((crc32 & 0xFF000000) >> 24)
				list_bytes.append((crc32 & 0xFF0000) >> 16)
				list_bytes.append((crc32 & 0xFF00) >> 8)
				list_bytes.append(crc32 & 0xFF)

				byt = ["0x%02x" % n for n in list_bytes]  	# DEBUG
				print(byt)  					# DEBUG hex
			#	print(list_bytes)				# DEBUG dec
				print("crc: " + str(crc32))  			# DEBUG

				status = REPEAT
				while(REPEAT == status):
					spi.writebytes(list_bytes)  		# SPI Write payload
					status = uart.read(1)
					uart.write(status)

		state = SUSPEND
# ----------------------------------------------------------------------------------------------------------------------
	else:
		print("Didn't match a case")
		print(state)
		state = SUSPEND
# ----------------------------------------------------------------------------------------------------------------------
