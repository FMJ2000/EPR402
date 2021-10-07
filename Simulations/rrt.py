import pygame, random, time, math

SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
BLOCK_WIDTH = 40

random.seed(time.time())

class Bot(pygame.sprite.Sprite):
	def __init__(self):
		self.image = pygame.image.load('bot.png')
		self.w, self.h = self.image.get_size()
		self.surf = self.image
		self.origin = (random.randrange(self.w, SCREEN_WIDTH - 2*self.w), random.randrange(self.h, SCREEN_HEIGHT - 2*self.h))
		self.rotation = 0
		self.angle_offset = math.atan2(self.w, self.h) * 180 / math.pi
		print(f'size: ({self.w}, {self.h}, {self.angle_offset})')

	def rrt(self):
		

	def getPos(self):
		self.w, self.h = self.surf.get_size()
		angle = (self.rotation + self.angle_offset) * math.pi / 180.0
		pos = (self.origin[0] - 0.5 * self.w * math.sin(angle), self.origin[1] - 0.5 * self.h * math.cos(angle))
		print(f'origin: {self.origin}, pos: {pos}')
		return pos

	def rotate(self, angle):
		self.rotation += angle
		self.surf = pygame.transform.rotate(self.image, self.rotation)

class Obstacle(pygame.sprite.Sprite):
	def __init__(self, pos):
		super(Obstacle, self).__init__()
		self.surf = pygame.Surface((BLOCK_WIDTH, BLOCK_WIDTH))
		self.surf.fill((255, 255, 255))
		self.rect = self.surf.get_rect()
		self.pos = pos

pygame.init()
screen = pygame.display.set_mode([SCREEN_WIDTH, SCREEN_HEIGHT])
pygame.display.set_caption('Hendrik RRT Simulation')
clock = pygame.time.Clock()

bot = Bot()
# create walls
obstacles = []
for i in range(0, SCREEN_WIDTH, BLOCK_WIDTH):
	obstacles.append(Obstacle((i, 0)))
	obstacles.append(Obstacle((i, SCREEN_HEIGHT - BLOCK_WIDTH)))
for i in range(0, SCREEN_HEIGHT, BLOCK_WIDTH):
	obstacles.append(Obstacle((0, i)))
	obstacles.append(Obstacle((SCREEN_WIDTH - BLOCK_WIDTH, i)))
for i in range(6):
	obstacles.append(Obstacle((random.randrange(BLOCK_WIDTH, SCREEN_WIDTH - 2*BLOCK_WIDTH), random.randrange(BLOCK_WIDTH, SCREEN_HEIGHT - 2*BLOCK_WIDTH))))

running = True
while running:
	for event in pygame.event.get():
		if event.type == pygame.QUIT:
			running = False
	
	screen.fill((0, 0, 0))
	for obstacle in obstacles:
		screen.blit(obstacle.surf, obstacle.pos)
	screen.blit(bot.surf, bot.getPos())

	pygame.display.update()
	clock.tick(15)

pygame.quit()