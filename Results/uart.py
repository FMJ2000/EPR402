import serial, csv, time

ser = serial.Serial(port='/dev/rfcomm0', baudrate=9600)
print(ser.name)
ser.flushInput()

while True:
	try:
		ser_bytes = ser.readline()
		decoded_bytes = ser_bytes.decode('utf-8')
		print(decoded_bytes, end='')
		with open('explore_log1.txt', 'a') as f:
			f.write(decoded_bytes)
	except Exception as e:
		pass
	current = time.time()
