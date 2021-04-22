import os, time, random

bot_rotations = ['^ ', '> ', 'v ', '< ']

# world class contains real map and bot position
class World:
	# width and height for map, x, y and rot for bot: rot = [up, right, down, left]
	def __init__(self, width, height, x, y, r):
		self.map = self.init_map(width, height)
		self.bot = {
			'x': x,
			'y': y,
			'r': r	
		}

	# construct initial map
	def init_map(self, width, height):
		map = [[0 for _ in range(width)] for _ in range(height)]

		# four walls
		for i in range(width):
			map[0][i] = 255
			map[-1][i] = 255
		for i in range(1, height - 1):
			map[i][0] = 255
			map[i][-1] = 255

		# defining features
		for i in range(1, 5):
			map[i][4] = 255
		
		for i in range(2, 5):
			map[-i][8] = 255

		return map

	# returns what the bot is seeing from the world
	# output left to right or top to bottom
	def view(self):
		output = [-1, -1, -1]
		if self.bot['r'] == 0:
			if self.bot['y'] - 1 >= 0:
				output[1] = self.map[self.bot['y'] - 1][self.bot['x']]
				if self.bot['x'] - 1 >= 0:
					output[0] = self.map[self.bot['y'] - 1][self.bot['x'] - 1]
				if self.bot['x'] + 1 < len(self.map[0]):
					output[2] = self.map[self.bot['y'] - 1][self.bot['x'] + 1]
		elif self.bot['r'] == 1:
			if self.bot['x'] + 1 < len(self.map[0]):
				output[1] = self.map[self.bot['y']][self.bot['x'] + 1]
				if self.bot['y'] - 1 >= 0:
					output[0] = self.map[self.bot['y'] - 1][self.bot['x'] + 1]
				if self.bot['y'] + 1 < len(self.map):
					output[2] = self.map[self.bot['y'] + 1][self.bot['x'] + 1]
		elif self.bot['r'] == 2:
			if self.bot['y'] + 1 < len(self.map):
				output[1] = self.map[self.bot['y'] + 1][self.bot['x']]
				if self.bot['x'] - 1 >= 0:
					output[0] = self.map[self.bot['y'] + 1][self.bot['x'] - 1]
				if self.bot['x'] + 1 < len(self.map[0]):
					output[2] = self.map[self.bot['y'] + 1][self.bot['x'] + 1]
		elif self.bot['r'] == 3:
			if self.bot['x'] - 1 >= 0:
				output[1] = self.map[self.bot['y']][self.bot['x'] - 1]
				if self.bot['y'] - 1 >= 0:
					output[0] = self.map[self.bot['y'] - 1][self.bot['x'] - 1]
				if self.bot['y'] + 1 < len(self.map):
					output[2] = self.map[self.bot['y'] + 1][self.bot['x'] - 1]
		return output

	def forward(self):
		if self.bot['r'] == 0:
			self.bot['y'] -= 1
		elif self.bot['r'] == 1:
			self.bot['x'] += 1
		elif self.bot['r'] == 2:
			self.bot['y'] += 1
		elif self.bot['r'] == 3:
			self.bot['x'] -= 1

	# left = -1, right = 1
	def turn(self, side):
		self.bot['r'] = (self.bot['r'] + side) % 4

class Bot:
	def __init__(self, width, height, r):
		self.pos = {
			'x': width // 2,
			'y': height // 2,
			'r': r
		}
		self.map = [[0 for _ in range(width)] for _ in range(height)]

	def see(self, input):
		if self.pos['r'] == 0:
			if input[0] != -1:
				self.map[self.pos['y'] - 1][self.pos['x'] - 1] = input[0]
			if input[1] != -1:
				self.map[self.pos['y'] - 1][self.pos['x']] = input[1]
			if input[2] != -1:
				self.map[self.pos['y'] - 1][self.pos['x'] + 1] = input[2]
		elif self.pos['r'] == 1:
			if input[0] != -1:
				self.map[self.pos['y'] - 1][self.pos['x'] + 1] = input[0]
			if input[1] != -1:
				self.map[self.pos['y']][self.pos['x'] + 1] = input[1]
			if input[2] != -1:
				self.map[self.pos['y'] + 1][self.pos['x'] + 1] = input[2]
		elif self.pos['r'] == 2:
			if input[0] != -1:
				self.map[self.pos['y'] + 1][self.pos['x'] - 1] = input[0]
			if input[1] != -1:
				self.map[self.pos['y'] + 1][self.pos['x']] = input[1]
			if input[2] != -1:
				self.map[self.pos['y'] + 1][self.pos['x'] + 1] = input[2]
		elif self.pos['r'] == 3:
			if input[0] != -1:
				self.map[self.pos['y'] - 1][self.pos['x'] - 1] = input[0]
			if input[1] != -1:
				self.map[self.pos['y']][self.pos['x'] - 1] = input[1]
			if input[2] != -1:
				self.map[self.pos['y'] + 1][self.pos['x'] - 1] = input[2]

	def move(self, world):
		if self.get_front() != -1:
			world.forward()
			self.forward()
		else:
			turn = random.randint(0, 1) * 2 - 1
			world.turn(turn)
			self.turn(turn)

	def forward(self):
		if self.pos['r'] == 0:
			self.pos['y'] -= 1
		elif self.pos['r'] == 1:
			self.pos['x'] += 1
		elif self.pos['r'] == 2:
			self.pos['y'] += 1
		elif self.pos['r'] == 3:
			self.pos['x'] -= 1

	# left = -1, right = 1
	def turn(self, side):
		self.pos['r'] = (self.pos['r'] + side) % 4

	def get_front(self):
		if self.pos['r'] == 0 and self.map[self.pos['y'] - 1][self.pos['x']] != 255:
			return self.map[self.pos['y'] - 1][self.pos['x']]
		if self.pos['r'] == 1 and self.map[self.pos['y']][self.pos['x'] + 1] != 255:
			return self.map[self.pos['y']][self.pos['x'] + 1]
		if self.pos['r'] == 2 and self.map[self.pos['y'] + 1][self.pos['x']] != 255:
			return self.map[self.pos['y'] + 1][self.pos['x']]
		if self.pos['r'] == 3 and self.map[self.pos['y']][self.pos['x'] - 1] != 255:
			return self.map[self.pos['y']][self.pos['x'] - 1]
		return -1

def print_map(world, bot):
	os.system('clear')
	print('World Map:\n')

	for i in range(len(world.map)):
		output = ''
		for j in range(len(world.map[i])):
			if world.bot['x'] == j and world.bot['y'] == i:
				output += bot_rotations[world.bot['r']]
			else:
				output += '# ' if world.map[i][j] == 255 else '  '
		print(output)

	print('Bot Map:\n')
	for i in range(len(bot.map)):
		output = ''
		for j in range(len(bot.map[i])):
			if bot.pos['x'] == j and bot.pos['y'] == i:
				output += bot_rotations[bot.pos['r']]
			else:
				output += '# ' if bot.map[i][j] == 255 else '  '
		print(output)

width = random.randint(15, 25)
height = random.randint(8, 15)
x = random.randint(1, width - 1)
y = random.randint(1, height - 1)
r = random.randint(0, 4)
world = World(width, height, x, y, r)
bot = Bot(width * 2, height * 2, r)
while True:
	bot.see(world.view())
	#print_map(world, bot)
	#time.sleep(0.5)
	bot.move(world)
	print_map(world, bot)
	time.sleep(0.2)