import os, time, random

bot_rotations = ['^ ', '> ', 'v ', '< ']

# world class contains real map and bot position
class World:
	# width and height for map
	def __init__(self, width, height, x, y, r):
		self.map = self.init_map(width, height)
		self.bot = {
			x: x,
			y: y,
			r: r
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

class Bot:
	def __init__(self, r):
		self.nodes = []
		self.x = 0
		self.y = 0
		self.r = r

	def sense(self, world):
		pass

def print_map(world, bot):
	os.system('clear')
	print('World Map:\n')
	print(world.bot)

	for i in range(len(world.map)):
		output = ''
		for j in range(len(world.map[i])):
			if world.bot.x == j and world.bot.y == i:
				output += bot_rotations[world.bot.r]
			else:
				output += '# ' if world.map[i][j] == 255 else '  '
		print(output)

	print('Bot Map:\n')
	for i in range(len(bot.map)):
		output = ''
		for j in range(len(bot.map[i])):
			if bot.x == j and bot.y == i:
				output += bot_rotations[bot.r]
			else:
				output += '# ' if bot.map[i][j] == 255 else '  '
		print(output)

width = random.randint(15, 25)
height = random.randint(8, 15)
x = random.randint(1, width - 1)
y = random.randint(1, height - 1)
r = random.randint(0, 4)
world = World(width, height, x, y, r)
bot = Bot(r)

while True:
	print_map(world, bot)
	time.sleep(0.2)