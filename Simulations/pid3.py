import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import math, random
np.set_printoptions(precision=2, suppress=True)

DT = 0.025
K_DO = 0.4
K_DA = 0.08
K_RO = np.pi
K_RA = 0.4
K_DP = 0.8
K_DI = 0.002
K_DD = 0.04
K_RP = 0.4
K_RI = 0.002
K_RD = 0.04
SCALE_ERR_R = 0.9
SCALE_ERR_L = 1.1
WHEEL_R = 0.032
CHASSIS_L = 0.15
PWM_V = 48

def normAngle(x):
	x = math.fmod(x + np.pi, 2*np.pi)
	if (x < 0):
		x += 2*np.pi
	return x - np.pi

plotPos = np.zeros((1,7))
pos = [0 for _ in range(7)]
goal = np.array([
	[0., 0.45],
	[-0.3, 0.2],
	[-0.45, -0.23],
	[0.45, 0.45]
])
ePos = [0 for _ in range(2)]
eVel = [0 for _ in range(2)]
ePosInt = [0 for _ in range(2)]
pid = [0 for _ in range(2)]
duty = [0 for _ in range(2)]
colors = cm.rainbow(np.linspace(0, 1, 4))
count = 0.

for i in range(4):
	for j in range(120):
		# controller
		if (j % 20 == 0):
			ePosInt[0] = 0
			ePosInt[1] = 0
		ePos[0] = np.sqrt((goal[i][0] - pos[0])**2 + (goal[i][1] - pos[1])**2)
		ePos[1] = np.arctan2(goal[i][1] - pos[1], goal[i][0] - pos[0]) - pos[2]
		ePos[1] = normAngle(ePos[1])
		#print(f'{ePos[1]:.2f} -> {normAngle(ePos[1]):.2f}')
		eVel[0] = K_DO * ePos[0] / (K_DA + ePos[0]) - pos[3]
		eVel[1] = K_RO * np.abs(ePos[1]) / (K_RA + np.abs(ePos[1])) - pos[5]
		ePosInt[0] += ePos[0] * DT
		ePosInt[1] += ePos[1] * DT

		pid[0] = K_DP*ePos[0] + K_DI*ePosInt[0] + K_DD*eVel[0]
		pid[1] = K_RP*ePos[1] + K_RI*ePosInt[1] + K_RD*eVel[1]
		
		duty[0] = K_DO * pid[0] / (K_DA + pid[0]) + pid[1]
		duty[1] = K_DO * pid[0] / (K_DA + pid[0]) - pid[1]
		if (duty[0] < 0.12):
			duty[0] = 0
		if (duty[1] < 0.12):
			duty[1] = 0
		if (j == 0):
			print(f'pos: ({pos[0]:.2f}, {pos[1]:.2f}), ePos: ({ePos[0]:.2f}, {ePos[1]:.2f}), pid: ({pid[0]:.2f} {pid[1]:.2f})')
		
		
		if (ePos[0] < 0.04):
			count += j*DT
			break
		
		# plant
		noise = np.random.normal(0, 0.1, 2)
		duty[0] = SCALE_ERR_R * duty[0] + noise[0]
		duty[1] = SCALE_ERR_L * duty[1] + noise[1]
		pos[3] = 0.5*WHEEL_R*PWM_V*np.cos(pos[2])*(duty[0] + duty[1])
		pos[4] = 0.5*WHEEL_R*PWM_V*np.sin(pos[2])*(duty[0] + duty[1])
		pos[5] = WHEEL_R*PWM_V*(duty[0] - duty[1]) / CHASSIS_L
		pos[0] += pos[3]*DT
		pos[1] += pos[4]*DT
		pos[2] += pos[5]*DT
		pos[6] = i
		plotPos = np.concatenate((plotPos, [pos]), axis=0)
		
print(f'total time: {count:.2f}')
		
# plot points
plt.xlim(-0.5, 0.5)
plt.ylim(-0.5, 0.5) 
plt.scatter(plotPos[:,0], plotPos[:,1])
plt.scatter(goal[:,0], goal[:,1])
plt.grid()
plt.show()
