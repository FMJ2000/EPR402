import serial, csv, time

ser = serial.Serial(port='/dev/ttyUSB0', baudrate=115200)
print(ser.name)
ser.flushInput()

while True:
	try:
		ser_bytes = ser.readline()
		decoded_bytes = ser_bytes.decode('utf-8')
		print(decoded_bytes, end='')
		with open('path_log.txt', 'a') as f:
			f.write(decoded_bytes)
	except Exception as e:
		pass
	current = time.time()
