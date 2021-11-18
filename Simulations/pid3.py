import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import math, random
np.set_printoptions(precision=2, suppress=True)

DT = 0.025
K_DO = 0.3
K_DA = 0.008
K_RO = 0.6
K_RA = 0.4

K_DP = 0.6
K_DI = 0.002
K_DD = 0.002
K_RP = 0.2
K_RI = 0.0002
K_RD = 0.001
K_UV = 0.2
K_UW = 0.2
SCALE_ERR_R = 0.9
SCALE_ERR_L = 1.1
WHEEL_R = 0.032
CHASSIS_L = 0.15
PWM_V = 48
DC_TO_ODO = 15
INTEGRAL_LEN = 40

def normAngle(x):
	x = math.fmod(x + np.pi, 2*np.pi)
	if (x < 0):
		x += 2*np.pi
	return x - np.pi

plotPos = np.zeros((1,6))
pos = [0 for _ in range(6)]
goal = np.array([
	[0.3, 0.3],#45],
	[-0.3, 0.2],
	[-0.45, -0.23],
	[0.45, 0.25],
	[0.1, -0.2]
])
ePos = [0 for _ in range(2)]
eVel = [0 for _ in range(2)]
ePosInt = [[0 for _ in range(2)] for _ in range(INTEGRAL_LEN)]
prevIndex = 0
pid = [0 for _ in range(2)]
uGoal = [0 for _ in range(2)]
eDuty = [0 for _ in range(2)]
duty = [0 for _ in range(2)]
odo = [0 for _ in range(2)]
colors = cm.rainbow(np.linspace(0, 1, 4))
count = 0.

for i in range(5):
	for j in range(120):
		# pos controller
		ePos[0] = np.sqrt((goal[i][0] - pos[0])**2 + (goal[i][1] - pos[1])**2)
		ePos[1] = normAngle(np.arctan2(goal[i][1] - pos[1], goal[i][0] - pos[0]) - pos[2])
		ePosInt[prevIndex][0] = ePos[0]
		ePosInt[prevIndex][1] = ePos[1]
		#print(f'{ePos[1]:.2f} -> {normAngle(ePos[1]):.2f}')
		eVel[0] = (ePos[0] - ePosInt[(prevIndex - 1) % INTEGRAL_LEN][0]) / DT
		eVel[1] = (ePos[1] - ePosInt[(prevIndex - 1) % INTEGRAL_LEN][1]) / DT
		#eVel[0] = K_DO * ePos[0] / (K_DA + ePos[0]) - pos[3]
		#eVel[1] = K_RO * np.abs(ePos[1]) / (K_RA + np.abs(ePos[1])) - pos[4]
		eInt = [0 for _ in range(2)]
		for k in range(INTEGRAL_LEN):
			eInt[0] += ePosInt[k][0] * DT
			eInt[1] += ePosInt[k][1] * DT
		prevIndex = (prevIndex + 1) % INTEGRAL_LEN

		pid[0] = K_DP*ePos[0] + K_DI*eInt[0] + K_DD*eVel[0]
		pid[1] = K_RP*ePos[1] + K_RI*eInt[1] + K_RD*eVel[1]

		uGoal[0] = K_DO * pid[0] / (K_DA + pid[0])
		uGoal[1] = K_RO * pid[1]# K_RO * pid[1] / (K_RA + pid[1])

		#eDuty[0] = uGoal[0] - pos[3]#0.5*WHEEL_R*PWM_V*(odo[0] + odo[1])
		#eDuty[1] = uGoal[1] - pos[4]#WHEEL_R*PWM_V*(odo[0] - odo[1]) / CHASSIS_L
		
		duty[0] = uGoal[0] + uGoal[1]#(K_UV*uGoal[0] + K_UW*uGoal[1]) / DC_TO_ODO
		duty[1] = uGoal[0] - uGoal[1]#(K_UV*uGoal[0] - K_UW*uGoal[1]) / DC_TO_ODO
		if (np.abs(duty[0]) < 0.12):
			duty[0] = 0
		if (np.abs(duty[1]) < 0.12):
			duty[1] = 0
		if (np.abs(duty[0]) > 1):
			duty[0] /= np.abs(duty[0])
		if (np.abs(duty[1]) > 1):
			duty[1] /= np.abs(duty[1])
		
		if (ePos[0] < 0.1):
			count += j*DT
			break
		
		# plant
		#noise = np.random.normal(0, 0.1, 2)
		#odo[0] = SCALE_ERR_R * duty[0] + noise[0]
		#odo[1] = SCALE_ERR_L * duty[1] + noise[1]
		odo[0] = 2*np.pi*DC_TO_ODO*duty[0]*WHEEL_R
		odo[1] = 2*np.pi*DC_TO_ODO*duty[1]*WHEEL_R
		pos[3] = 0.5*(odo[0] + odo[1])
		pos[4] = (odo[0] - odo[1]) / CHASSIS_L
		pos[2] = normAngle(pos[2] + pos[4]*DT)
		pos[0] += pos[3]*np.cos(pos[2])*DT
		pos[1] += pos[3]*np.sin(pos[2])*DT
		pos[5] = i
		plotPos = np.concatenate((plotPos, [pos]), axis=0)

		print(f'pos: ({pos[0]:.2f}, {pos[1]:.2f}, {(pos[2]*180/np.pi):.2f}, {pos[3]:.2f}, {(pos[4]*180/np.pi):.2f}), '
			f'ePos: ({ePos[0]:.2f}, {(ePos[1]*180/np.pi):.2f}), '
			f'eVel: ({eVel[0]:.2f}, {(eVel[1]*180/np.pi):.2f}), '
			f'eInt: ({eInt[0]:.2f}, {(eInt[1]*180/np.pi):.2f})\r\n'
			f'pid: ({pid[0]:.2f} {pid[1]:.2f}), '
			f'uGoal: ({uGoal[0]:.2f}, {uGoal[1]:.2f}), '
			f'eDuty: ({eDuty[0]:.2f}, {(eDuty[1]*180/np.pi):.2f}), '
			f'duty: ({duty[0]:.2f}, {duty[1]:.2f})\r\n'
			f'odo: ({odo[0]:.2f}, {odo[1]:.2f})\r\n'
		)
		#input()
		
print(f'total time: {count:.2f}')
		
# plot points
#plt.xlim(-0.5, 0.5)
#plt.ylim(-0.5, 0.5)
plt.scatter(plotPos[:,0], plotPos[:,1], s=100, marker='.', label='path')
plt.scatter(goal[:,0], goal[:,1], s=400, marker='*', label='goals')
plt.scatter(0, 0, s=400, marker='p', label='init pos')
plt.title('Robot using PID-controller in path following with noisy actuators')
plt.grid()
plt.legend()
plt.show()
