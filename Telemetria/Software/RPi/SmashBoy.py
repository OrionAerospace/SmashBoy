import spidev
import serial
from io import BytesIO
from time import sleep
from picamera import PiCamera
import os
import zlib

#	---=-=-= CONFIGURATION =-=-=---

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
spi.max_speed_hz = 1000000
spi.mode = 0

# Camera capture
#camera.capture("my_image.jpg")
my_file = open("pluto128.jpg", "rb")


#  ---=-=-= PAYLOAD =-=-=---


my_file.seek(0, os.SEEK_END)
size = my_file.tell()								# Size calculator
my_file.seek(0)

size_bytes = (size).to_bytes(4, byteorder='big')
list_size_bytes = list(size_bytes)					# Size to metaData
list_size_bytes.insert(0, 0x4D)						# Identificador Metadado


print("size: " + str(size))							# DEBUG
#print("parts: " + str(int(size/2024)) )			# DEBUG


print("aguardando comando 1")						# DEBUG
command = ser.read(1)								# Aguarda pelo comando de metadata
while command != b'\x01':							# Enquanto for diferente, fica aguardando aqui
	
	if command == b'\x55':							# 0x55 é o comando de reset do algoritmo
		ser.write(b'\x55')							# Cotejamento
		print("break 1")							# DEBUG
		break										# Sai do while
	
	ser.write(b'\xFF')								# 0xFF informa que o comando esta errado, espera-se que isso nunca ocorra
	command = ser.read(1)							# Aguarda pelo comando


if command == b'\x01':
	ser.write(b'\x01')								# Cotejamento
	spi.writebytes(list_size_bytes)					# Write Full size os file (metadata)

	print("metadado enviado")						# DEBUG

	loraPyldMatrix = 253 * 16						# 253 Bytes = Payload LoRa, 253*16 = 4048 Bytes Payload Buffer STM32
	ID = int(size/loraPyldMatrix)

	print("aguardando comando 2")					# DEBUG
	command = ser.read(1)							# Aguarda pelo comando de iniciar envio do pacote
	while command != b'\x02':							
		
		if command == b'\x55':						# 0x55 reset 
			ser.write(b'\x55')						# Cotejamento
			print("break 2")						# DEBUG
			break									# Sai do while
		
		ser.write(b'\xFF')							# 0xFF erro
		command = ser.read(1)						# Aguarda pelo comando novamente

	if command == b'\x02':
		ser.write(b'\x02')							# Cotejamento

		for i in range(ID):
			list_bytes = list(my_file.read(loraPyldMatrix))			# Payload (image)
			list_bytes.insert(0, loraPyldMatrix & 0xFF)				# Size L
			list_bytes.insert(0, (loraPyldMatrix >> 8) & 0xFF)		# Size H
			list_bytes.insert(0,  (ID - i) & 0xFF)					# ID L  (part)
			list_bytes.insert(0, ((ID - i) & 0xFF00) >> 8)			# ID H

			crc32 = zlib.crc32( bytes(list_bytes) )					# CRC32
			list_bytes.append( (crc32 & 0xFF000000) >> 24 )
			list_bytes.append( (crc32 & 0xFF0000) >> 16 )
			list_bytes.append( (crc32 & 0xFF00) >> 8 )
			list_bytes.append(  crc32 & 0xFF )

			byt = ["0x%02x" % n for n in list_bytes]		# DEBUG
			print(byt)										# DEBUG hex
		#	print(list_bytes)								# DEBUG dec
			print("crc: " + str(crc32))						# DEBUG

			spi.writebytes(list_bytes)						# SPI Write payload

			print("aguardando comando 3")					# DEBUG
			command = ser.read(1)
			while command != b'\x03':						# wait for continue
				
				if command == b'\x00':						# repete mensagem, deve ter dado CRC FAIL no uC
					print("repetindo..")					# DEBUG
					spi.writebytes(list_bytes)

				elif command == b'\x55':					# 0x55 reset 
					ser.write(b'\x55')						# Cotejamento
					print("break 3.1")						# DEBUG
					break									# Sai do while

				ser.write(b'\xFF')							# 0xFF erro
				command = ser.read(1)						# Aguarda pelo comando novamente

			if command == b'\x55':							# Sai do loop for
				print("break 3.2")
				break

		else:
			remainder = size - my_file.tell()				# remainder payload
			print("resto: " + str(remainder))				# DEBUG

			list_bytes = list( my_file.read(remainder) )
			list_bytes.insert(0, remainder & 0xFF)					# Size L
			list_bytes.insert(0, (remainder & 0xFF00) >> 8)			# Size H
			list_bytes.insert(0, 0)									# ID L (part)
			list_bytes.insert(0, 0)									# ID H

			crc32 = zlib.crc32( bytes(list_bytes) )					# CRC32
			list_bytes.append( (crc32 & 0xFF000000) >> 24 )
			list_bytes.append( (crc32 & 0xFF0000) >> 16 )
			list_bytes.append( (crc32 & 0xFF00) >> 8 )
			list_bytes.append(  crc32 & 0xFF )

			byt = ["0x%02x" % n for n in list_bytes]		# DEBUG
			print(byt)										# DEBUG hex
		#	print(list_bytes)								# DEBUG dec
			print("crc: " + str(crc32))						# DEBUG

			spi.writebytes(list_bytes)						# SPI Write payload

			print("aguardando comando 3.1")					# DEBUG
			command = ser.read(1)
			while command != b'\x03':						# wait for continue
				
				if command == b'\x00':						# repete mensagem, deve ter dado CRC FAIL no uC
					print("repetindo..")					# DEBUG
					spi.writebytes(list_bytes)

				elif command == b'\x55':					# 0x55 reset 
					ser.write(b'\x55')						# Cotejamento
					print("break 4.1")						# DEBUG
					break									# Sai do while

				ser.write(b'\xFF')							# 0xFF erro
				command = ser.read(1)						# Aguarda pelo comando novamente


# close port
ser.close()
spi.close()
#camera.close()
my_file.close()

print("THE END")											# DEBUG