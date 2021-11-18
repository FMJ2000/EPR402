import numpy as np
import matplotlib.pyplot as plt
np.set_printoptions(precision=2, suppress=True)

STEP_SIZE = 0.2
MIN_GOAL_DIST = np.sqrt(2*STEP_SIZE**2)
MIN_OBST_DIST = 0.5

posMod = [
	[-1, 1],
	[0, 1],
	[1, 1],
	[1, 0],
	[1, -1],
	[0, -1],
	[-1, -1],
	[-1, 0]
]

np.random.seed(1)
obs = 4*np.random.rand(20, 2) - 2
goal = np.array([-2.04, 2.24])
init = np.array([0, -1])
path = np.array([init])
searchSpace = np.array([init])

class Node:
	def __init__(self, pos, posf, g, h, parent):
		self.pos = pos
		self.posf = posf
		self.g = g
		self.h = h
		self.parent = parent

def euclid(pos1, pos2):
	return np.sqrt((pos2[0] - pos1[0])**2 + (pos2[1] - pos1[1])**2)

def contains(list, pos):
	for i in range(len(list)):
		if list[i].pos[0] == pos[0] and list[i].pos[1] == pos[1]:
			return i
	return -1

def backtrack(node):
	global path
	path = np.array([node.posf])
	print(f'node: {node.posf}')
	count = 0
	while node.parent != None:
		node = node.parent
		if (count % 2):
			path = np.concatenate((path, [node.posf]))
		count += 1

def a_star():
	global path, searchSpace
	initNode = Node([0, 0], [init[0], init[1]], 0, euclid(init, goal), None)
	openQueue = [initNode]
	closedQueue = []
	
	iter = 0
	while len(openQueue) > 0 and iter < 1000:
		iter = iter+1
		node = openQueue[0]
		index = 0
		for i in range(1, len(openQueue)):
			if ((openQueue[i].g + openQueue[i].h) < (node.g + node.h)):
				node = openQueue[i]
				index = i
		openQueue.remove(openQueue[index])

		if node != None:
			searchSpace = np.concatenate((searchSpace, [node.posf]))
			closedQueue.append(node)
			if node.h < MIN_GOAL_DIST:
				backtrack(node)
				return

		for i in range(8):
			posI = [node.pos[0] + posMod[i][0], node.pos[1] + posMod[i][1]]
			fC = contains(closedQueue, posI)
			if fC == -1:
				posF = [init[0] + posI[0] * STEP_SIZE, init[1] + posI[1] * STEP_SIZE]
				fC = 0
				for j in range(len(obs)):
					if euclid(obs[j], posF) < MIN_OBST_DIST:
						fC = 1
						break
				if not fC:
					fC = contains(openQueue, posI)
					cost = np.sqrt(2*STEP_SIZE) if (i % 2) == 0 else STEP_SIZE 
					if fC == -1:	
						newNode = Node(posI, posF, node.g + cost, euclid(posF, goal), node)
						openQueue.append(newNode)
					elif (node.g + cost) < openQueue[fC].g:
						openQueue[fC].g = node.g + cost
						openQueue[fC].parent = node

	backtrack(node)
	#path = np.array([i.posf for i in closedQueue])
		
def plot():
	plt.scatter(searchSpace[:,0], searchSpace[:,1], s=600, c='lightblue', label='search space')
	plt.scatter(obs[:,0], obs[:,1], s=600, c='brown', label='obstacle', marker='s')
	plt.scatter(goal[0], goal[1], s=600, c='orange', label='goal', marker='*')
	plt.scatter(init[0], init[1], s=600, c='darkblue', label='initial pos', marker='p')
	plt.scatter(path[:,0], path[:,1], s=600, c='blue', label='path', marker='.')
	plt.xlim(-2.5, 2.5)
	plt.ylim(-2.5, 2.5)
	plt.title('A* search')
	plt.grid()
	plt.legend()
	plt.show()

if __name__ == '__main__':
	a_star()
	plot()