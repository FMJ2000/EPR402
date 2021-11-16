import numpy as np
np.set_printoptions(precision=2, suppress=True)

FREQ = 40
DT = 1 / FREQ

x = np.array([0, 0, 0, 1, np.pi/2])
x_ = np.array([
	x[0] + x[3]*np.cos(x[2]+x[4]),
	x[1] + x[3]*np.sin(x[2]+x[4]),
	x[2] + x[4],
	x[3],
	x[4]
])

y = np.array(x)
for i in range(FREQ):
	y[0] += y[3]*DT*np.cos(y[2]*y[4])
	y[1] += y[3]*DT*np.sin(y[2]*y[4])
	y[2] += y[4]*DT

print(f'x: {x}')
print(f'x_: {x_}')
print(f'y: {y}')
