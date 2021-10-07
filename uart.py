import serial

MSG_LEN = 26
US_SENSORS = 4

ser = serial.Serial(port='/dev/ttyUSB0', baudrate=38400)
print(ser.name)
ser.flushInput()

# data = { axh, axl, ayh, ayl, azh, azl, th, tl, gxh, gxl, gyh, gyl, gzh, gzl, v1h, v1l, v2h, v2l, v3h, v3l, v4h, v4l, odl, odr, bat, usr }
while True:
	try:
		data = list(ser.read(MSG_LEN))
		acc = [((data[2*i] << 8) | data[1+2*i]) for i in range(3)]
		gyro = [((data[8+2*i] << 8) | data[9+2*i]) for i in range(3)]
		view = [((data[14+2*i] << 8) | data[15+2*i]) / 30 for i in range(US_SENSORS)]
		print(f'1: {view[0]:.2f}cm, 2: {view[1]:.2f}cm, 3: {view[2]:.2f}cm, 4: {view[3]:.2f}cm | {data[0:14]}')
		#print(f'1: {view[0]:.2f}cm, 2: {view[1]:.2f}cm, 3: {view[2]:.2f}cm, 4: {view[3]:.2f}cm')
	except Exception as e:
		print(e)
