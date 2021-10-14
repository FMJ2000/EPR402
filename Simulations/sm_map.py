import numpy as np, time

SUBMAP_SIZE = 60
SUBMAP_RES = 3
SUBMAP_SIZE_U = SUBMAP_SIZE // SUBMAP_RES
BOT_SIZE = 10
EDGE_TRIGGER = 5
SENSORS = 3
SENSOR_MAX_DIST = 150
SENSOR_OFFSET = 30. * np.pi / 180.0
SENSOR_DEV = 15. * np.pi / 180.0
SIGMA = 3

# perform nibble multiplication on maps
def map_mul(map1, map2):
	for i in range(len(map1)):
		for j in range(len(map1[i])):
			map1[i][j] = (map1[i][j] + map2[i][j]) >> 1
	return map1

# get actions and distances sensed from bot
def read_actions(master):
	with open('data.csv') as file:
		lines = file.readlines()
		lines = [[float(i) for i in line.split(',')] for line in lines]
		for line in lines:
			pos = line[:3]
			pos[2] *= np.pi / 180.0
			distance = line[3:]
			print(f'action: {pos}, distance: {distance}')
			master.update_pos(pos)
			master.send_map()
			master.slave.distance_reading(distance)
			master.update_map()
			input()
	master.print_map(master.map, True)

class Map:
	def __init__(self, pos, size, default=0x7):
		self.pos = pos
		self.size_cm = size
		self.size_u = (size[0] // SUBMAP_RES, size[1] // SUBMAP_RES)
		self.map = np.full(self.size_u, default)
		self.neigbors = [-1 for _ in range(8)]

	def map_contains(self, pos):
		xrange = (self.pos[0], self.pos[0] + self.size_cm[0])
		yrange = (self.pos[1], self.pos[1] + self.size_cm[1])
		if pos[0] < xrange[0] or pos[0] >= xrange[1] or pos[1] < yrange[0] or pos[1] >= yrange[1]:
			return False
		return True

	# set value of position
	def set_val(self, pos, val):
		# convert from centimeters to units
		index = (int((pos[1] - self.pos[1]) / SUBMAP_RES), int((pos[0] - self.pos[0]) / SUBMAP_RES))
		self.map[index[0], index[1]] = val

	def print_map(self, pos=None, symbolic=False):
		print(f'pos: {self.pos}')
		if pos:
			print(f'bot pos: {pos}')
		if not symbolic:
			display_map = self.map
			if pos and self.map_contains(pos):
				display_map[int((pos[1] - self.pos[1]) / SUBMAP_RES)][int((pos[0] - self.pos[0]) / SUBMAP_RES)] = 0x0
			with np.printoptions(formatter={'int':'{:X}'.format}):
				print(display_map)
		else:
			symbol = np.full(self.size_u, ' ',)
			for i in range(len(self.map)):
				for j in range(len(self.map[i])):
					if pos and pos[0] == self.pos[0] + i and pos[1] == self.pos[1] + j:
						print(f'i: {i}, j: {j}')
						symbol[i][j] = '@'
					elif self.map[i][j] >= 0x7:
						symbol[i][j] = '$'
			print(symbol)

class Slave:
	def __init__(self, master):
		self.master = master
		self.pos = (0., 0., 0.)
		self.map_size = (SUBMAP_SIZE, SUBMAP_SIZE)
		self.map = Map(self.pos, self.map_size)
		#self.world_map = read_world()
		self.offset = np.array([-SENSOR_OFFSET, 0., SENSOR_OFFSET])
		self.corr = 0
		self.sigma = [SIGMA, SIGMA]

		#self.world_map.print_map()

	def update_pos(self, pos):
		self.pos = pos
		self.angle_a = (self.pos[2] - SENSOR_OFFSET - SENSOR_DEV) % (2*np.pi)
		self.angle_b = (self.pos[2] - SENSOR_DEV) % (2*np.pi)
		self.angle_c = (self.pos[2] + SENSOR_DEV) % (2*np.pi)
		self.angle_d = (self.pos[2] + SENSOR_OFFSET + SENSOR_DEV) % (2*np.pi)

	# receive submap and bot position, orientation in submap
	def receive_map(self, pos, map):
		self.update_pos(pos)
		self.map = map
		#self.map.print_map(self.pos)

	# convert distance reading to position on submap
	def distance_to_pos(self, distance):
		pos_matrix = np.array([self.pos[:2] for _ in range(SENSORS)]).T
		angle = np.array([np.cos([self.offset[i] - self.pos[2] for i in range(SENSORS)]), np.sin([self.offset[i] - self.pos[2] for i in range(SENSORS)])])
		pos_matrix = (pos_matrix + np.matmul(angle, np.diag(distance))).T.tolist()
		pos = []
		for i in range(len(pos_matrix)):
			if self.map.map_contains(pos_matrix[i]):
				pos.append(pos_matrix[i])
		return pos

	# test if position is visible to bot
	def view_contains(self, pos, distances):
		# get angle relative to bot
		if pos[0] == self.pos[0]:
			pos[0] += 0.001
		frac = -(pos[1] - self.pos[1]) / (pos[0] - self.pos[0])
		angle = (np.arctan(frac))
		if (pos[0] - self.pos[0]) < 0:
			angle = np.pi + angle
		dist = np.sqrt((pos[0] - self.pos[0])**2 + (pos[1] - self.pos[1])**2)

		angle = angle % (2*np.pi)
		a = self.angle_a
		b = self.angle_b
		c = self.angle_c
		d = self.angle_d
		if a > d:
			offset = 2*np.pi - a
			a = (a + offset) % (2*np.pi)
			b = (b + offset) % (2*np.pi)
			c = (c + offset) % (2*np.pi)
			d = (d + offset) % (2*np.pi)
			angle = (angle + offset) % (2*np.pi)
		if a <= angle and angle <= b:
			return (dist <= distances[2] + 2*SIGMA)
		if b <= angle and angle <= c:
			return (dist <= distances[1] + 2*SIGMA)
		if c <= angle and angle <= d:
			return (dist <= distances[0] + 2*SIGMA)
		return False
		#print(f'pos: {pos}, angle: {(angle * 180/np.pi):.2f}, a: {(a * 180/np.pi):.2f}, b: {(b * 180/np.pi):.2f}, view: {(a <= angle and angle <= b)}')

	# get the multivariate gaussian distribution given means and covariance on data
	def multivariate_gaussian(self, mean, sigma, corr, pos):
		dev_x = (pos[0] - mean[0]) / sigma[0]
		dev_y = (pos[1] - mean[1]) / sigma[1]
		return 1 / (2*np.pi * sigma[0] * sigma[1] * np.sqrt(1 - corr**2)) * np.exp(-1 / (2 * (1 - corr**2)) * (dev_x**2 - 2*corr * dev_x * dev_y + dev_y**2))

	# construct obstacle probability map based on given means
	def probability_map(self, mean, distances):
		map = Map(self.map.pos, self.map_size)
		start_pos = [int(self.map.pos[0]), int(self.map.pos[1])]
		max = 0

		# inclusion and initial floating point map
		view_map = np.full(np.shape(map.map), False)
		float_map = np.full(np.shape(map.map), 0.0)

		for i in range(SUBMAP_SIZE_U):
			for j in range(SUBMAP_SIZE_U):
				pos = [start_pos[0] + SUBMAP_RES*i, start_pos[1] + SUBMAP_RES*j]
				view_map[j][i] = self.view_contains(pos, distances)
				if view_map[j][i]:
					for k in range(len(mean)):
						float_map[j][i] += self.multivariate_gaussian(mean[k], self.sigma, self.corr, pos)
					#print(float_map[j][i])
					if max < float_map[j][i]:
						max = float_map[j][i]
		max *= 1.01

		for i in range(SUBMAP_SIZE_U):
			for j in range(SUBMAP_SIZE_U):
				pos = [start_pos[0] + SUBMAP_RES*j, start_pos[1] + SUBMAP_RES*i]
				if view_map[i][j]:
					map.map[i][j] = int(float_map[i][j] * 16 / max)
				#print(f'{float_map[i][j]} -> {map.map[i][j]}')
		return map

	def distance_reading(self, distances):
		start = time.time()
		# distances outside submap scope are not taken into account
		#distances = np.array([30., 20., 40.])
		pos = self.distance_to_pos(distances)
		if len(pos) > 0:
			modifier = self.probability_map(pos, distances)
			self.map.map = map_mul(self.map.map, modifier.map)
			self.map.print_map(self.pos)

		# check speed of function
		end = time.time()
		print(f'time elapsed: {((end - start)*1000):.3f} ms')

	def post_map(self):
		return self.map

class Master:
	def __init__(self):
		self.slave = Slave(self)
		self.pos = (0., 0., 0.)
		self.map_size = (SUBMAP_SIZE, SUBMAP_SIZE)
		self.map = [Map((-self.map_size[0] // 2, -self.map_size[1] // 2), self.map_size)]
		self.current_submap = 0
		self.slave_pos = (0., 0., 0.)
		self.slave_index = -1

		# modify position and neighbor dict
		self.modify = {
			0: (1, 0),
			1: (1, -1),
			2: (0, -1),
			3: (-1, -1),
			4: (-1, 0),
			5: (-1, 1),
			6: (0, 1),
			7: (1, 1)
		}

	def rot_index(self, angle=None):
		if angle is None:
			angle = self.pos[2]
		return int((angle * 4 / np.pi) + 0.5) % 8
	
	# check if submap 1 and 2 are near and how
	def submap_near(self, map1, map2):
		for i in range(8):
			if map2.pos[0] == map1.pos[0] + self.modify[i][0] * SUBMAP_SIZE and map2.pos[1] == map1.pos[1] + self.modify[i][1] * SUBMAP_SIZE:
				return i
		return -1

	def reinforce_neighbors(self):
		for i in range(len(self.map)-1):
			for j in range(i+1, len(self.map)):
				index = self.submap_near(self.map[i], self.map[j])
				if index != -1:
					self.map[i].neigbors[index] = j
					self.map[j].neigbors[(index + 4) % 8] = i

	# used to check for new submaps and custom submaps
	def req_submaps(self, submap, trigger, offset=(0., 0.)):
		xrange = (submap.pos[0], submap.pos[0] + submap.size_cm[0])
		yrange = (submap.pos[1], submap.pos[1] + submap.size_cm[1])
		req = [0 for _ in range(8)]			# required submaps (0, 7) starting at positive x

		# test cardinal points
		if self.pos[0] + trigger + offset[0] >= xrange[1]:
			req[0] = 1
		if self.pos[1] - trigger + offset[1] < yrange[0]:
			req[2] = 1
		if self.pos[0] - trigger + offset[0] < xrange[0]:
			req[4] = 1
		if self.pos[1] + trigger + offset[1] >= yrange[1]:
			req[6] = 1
		
		# test if diagonal additional also required
		if req[0] and req[2]:
			req[1] = 1
		if req[2] and req[4]:
			req[3] = 1
		if req[4] and req[6]:
			req[5] = 1
		if req[6] and req[0]:
			req[7] = 1

		return req

	# master checks if it is necessary to add new submap to map
	def create_submap(self):
		curr_submap = [self.map[self.current_submap]]
		rot_i = self.rot_index()
		req = self.req_submaps(curr_submap[0], SUBMAP_SIZE // 2, (self.modify[rot_i][0] * BOT_SIZE, self.modify[rot_i][1] * BOT_SIZE))

		# test if submaps already exist
		for i in range(8):
			if req[i]:
				if curr_submap[0].neigbors[i] == -1:
					# map required and not available
					pos = (curr_submap[0].pos[0] + self.modify[i][0] * SUBMAP_SIZE, curr_submap[0].pos[1] + self.modify[i][1] * SUBMAP_SIZE)
					newMap = Map(pos, self.map_size)
					self.map.append(newMap)
					curr_submap.append(newMap)
				else:
					curr_submap.append(self.map[curr_submap[0].neigbors[i]])
		
		self.reinforce_neighbors()
		return curr_submap

	# create a copy of local submaps at intersection of current bot position and send to slave
	# first create larger map with 1, 2 or 4 associated submaps
	def send_map(self):
		maps = self.create_submap()

		# order maps geographically
		for i in range(len(maps)-1):
			for j in range(i+1, len(maps)):
				if (maps[i].pos[1] > maps[j].pos[1]) or (maps[i].pos[1] == maps[j].pos[1] and maps[i].pos[0] > maps[j].pos[0]):
					temp = maps[i]
					maps[i] = maps[j]
					maps[j] = temp
		
		big_map = maps[0].map
		if len(maps) > 1:
			big_map = np.concatenate((big_map, maps[1].map), axis=1)
		if len(maps) > 2:
			new_row = np.concatenate((maps[2].map, maps[3].map), axis=1)
			big_map = np.concatenate((big_map, new_row), axis=0)

		# send submap to slave relative to position and orientation
		rot_i = self.rot_index()
		self.slave_pos = (self.pos[0] - SUBMAP_SIZE // 2 + self.modify[rot_i][0] * (SUBMAP_SIZE // 2 - BOT_SIZE), self.pos[1] - SUBMAP_SIZE // 2 + self.modify[rot_i][1] * (SUBMAP_SIZE // 2 - BOT_SIZE))
		start_index = (int((self.slave_pos[0] - maps[0].pos[0]) / SUBMAP_RES) + 1, int((self.slave_pos[1] - maps[0].pos[1]) / SUBMAP_RES) + 1)
		slave_map = Map(self.slave_pos, self.map_size)
		for i in range(len(self.map)):
			if self.map[i].pos[0] == maps[0].pos[0] and self.map[i].pos[1] == maps[0].pos[1]:
				self.slave_index = i
				break
		slave_map.map = big_map[start_index[1]:start_index[1]+SUBMAP_SIZE_U, start_index[0]:start_index[0]+SUBMAP_SIZE_U]
		self.slave.receive_map(self.pos, slave_map)

	def update_pos(self, pos):
		self.pos = pos
		self.slave.update_pos(pos)

	# get slave map and update relevant submaps
	def update_map(self):
		slave_map = self.slave.post_map()
		top_map = self.map[self.slave_index]

		for i in range(slave_map.size_u[0]):
			for j in range(slave_map.size_u[1]):
				pos = [slave_map.pos[0] + SUBMAP_RES*j, slave_map.pos[1] + SUBMAP_RES*i]
				if top_map.map_contains(pos):
					top_map.set_val(pos, slave_map.map[i][j])
				elif self.map[top_map.neigbors[0]].map_contains(pos):
					self.map[top_map.neigbors[0]].set_val(pos, slave_map.map[i][j])
				elif self.map[top_map.neigbors[6]].map_contains(pos):
					self.map[top_map.neigbors[6]].set_val(pos, slave_map.map[i][j])
				elif self.map[top_map.neigbors[7]].map_contains(pos):
					self.map[top_map.neigbors[7]].set_val(pos, slave_map.map[i][j])


	def print_map(self, map=None, detail=True):
		if not map:
			map = self.map
		print(f'Submaps: {len(self.map)}')
		for submap in map:
			neighbor_pos = []
			for neighbor in submap.neigbors:
				if neighbor >= 0:
					neighbor_pos.append(map[neighbor].pos)
			print(f'Submap {submap.pos} -> {neighbor_pos}')
			if detail:
				submap.print_map(self.pos)

def main():
	np.set_printoptions(threshold=SUBMAP_SIZE**2, linewidth=150, precision=3)
	master = Master()
	read_actions(master)

if __name__ == '__main__':
	main()