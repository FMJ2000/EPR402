import numpy as np
import math
np.set_printoptions(precision=2, suppress=True)

K_RO = 0.9
K_RA = 0.5
FORWARD_CONST = 0.18

def normAngle(x):
	x = math.fmod(x + np.pi, 2*np.pi)
	if (x < 0):
		x += 2*np.pi
	return x - np.pi

def getAngle(pos1, pos2):
	denom = 0.001 if pos1[0] == pos2[0] else pos2[0] - pos1[0]
	return normAngle(np.arctan2(pos1[1] - pos2[1], denom) - pos1[2])

def getDistance(pos1, pos2):
	return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

pos = np.array([0, 0, -np.pi])
goal = np.array([-0.8, -0.1])
ePos = np.array([getDistance(pos, goal), getAngle(pos, goal)])
uGoal = np.array([0, K_RO * ePos[1]**2])
if uGoal[1] > FORWARD_CONST:
	uGoal[1] = np.sign(ePos[1]) * FORWARD_CONST
else:
	uGoal[1] *= np.sign(ePos[1])
uGoal[0] = FORWARD_CONST - abs(uGoal[1])

print(f'p: {pos}')
print(f'g: {goal}')
print(f'e: {ePos}')
print(f'u: {uGoal}')
