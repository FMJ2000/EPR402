import math, random, time

class Node():
	def __init__(self, pos, parent):
		self.pos = pos
		self.edges = [parent]
		parent.edges.append(self)

class Bot():
	def __init__(self):
		self.vel = [0.0, 0.0]
		self.pos = [0.0, 0.0]
		self.rot = 0.0
		self.rot_sigma = 0.05
		self.Q = 0.005
		self.R = 0.005
		self.a = None
		self.w = None

		self.r_pos = [0.0, 0.0]
		self.e_rot_s = 0.0
		self.K_p = 0.2
		self.K_i = 0.001
		self.pwm = [0.0, 0.0]

		self.K_d = 0.5
		self.K_a = 0.1

		self.min_dist = 3.0
		self.max_dist = 30.0
		self.nodes = []

	def update(self, a, w, dt):
		# kalman filter for orientation estimation
		# process
		self.a = a
		self.w = w
		e_mu = self.rot + w[2] * dt
		e_sigma = self.rot_sigma + self.Q

		# measurement and Kalman gain
		z = math.atan(math.sqrt(a[0]**2 + a[1]**2) / a[2])
		K = e_sigma / (e_sigma + self.R)
		self.rot = e_mu + K * (z - e_mu)
		self.rot_sigma = e_sigma - K * e_sigma

		#print(f'e_mu: {e_mu:.2f}, e_sigma: {e_sigma:.2f}, z: {z:.2f}, K: {K:.2f}')

		# update position based on rotation
		self.vel[0] += a[0] * dt * math.cos(self.rot) - a[1] * dt * math.sin(self.rot)
		self.vel[1] += + a[0] * dt * math.sin(self.rot) + a[1] * dt * math.cos(self.rot)
		self.pos[0] += self.vel[0] * dt * math.cos(self.rot) - self.vel[1] * dt * math.sin(self.rot)
		self.pos[1] += self.vel[0] * dt * math.sin(self.rot) + self.vel[1] * dt * math.cos(self.rot)

	def pid(self, dt):
		# desired and error orientation
		r_rot = math.atan2(self.r_pos[1] - self.pos[1], self.r_pos[0] - self.pos[0])
		e_rot = r_rot - self.rot
		self.e_rot_s += e_rot * dt
		#print(f'rot: {self.rot:.2f}, r_rot: {r_rot:.2f}, e_rot: {e_rot:.2f}')

		# obtain PWM from PI controller
		self.pwm[0] = 0.5 - self.K_p * e_rot - self.K_i * self.e_rot_s
		self.pwm[1] = 0.5 + self.K_p * e_rot + self.K_i * self.e_rot_s

	# simulate how movement will impact sensor readings
	def plant(self):
		a = [random.gauss(0, self.R), random.gauss(0, self.R), random.gauss(1, self.R)]
		w = [random.gauss(0, self.R) for _ in range(3)]

		w[2] += self.K_d * (self.pwm[0] - self.pwm[1])
		a[0] += self.K_a * (self.pwm[0] + self.pwm[1]) * math.cos(w[2])
		a[1] += self.K_a * (self.pwm[0] + self.pwm[1]) * math.sin(w[2])
		
		return a, w


	def rrt(self):
		# test if new position required
		if math.sqrt((self.r_pos[0] - self.pos[0])**2 + (self.r_pos[1] - self.pos[1])**2) < self.min_dist:
			self.r_pos = (random.randrange(int(self.pos[0] - self.max_dist), int(self.pos[0] + self.max_dist)), random.randrange(int(self.pos[1] - self.max_dist), int(self.pos[1] + self.max_dist)))

	def print(self):
		print(f'({self.pos[0]:.2f}, {self.pos[1]:.2f}, {(self.rot * 180 / math.pi):.2f}) -> ({self.r_pos[0]:.2f}, {self.r_pos[1]:.2f}) | pwm: ({self.pwm[0]:.2f}, {self.pwm[1]:.2f}) | a: ({self.a[0]:.2f}, {self.a[1]:.2f}, {self.a[2]:.2f}), w: ({self.w[0]:.2f}, {self.w[1]:.2f}, {self.w[2]:.2f})')

def main():
	random.seed(time.time())
	bot = Bot()
	dt = 0.2
	#bot.r_pos = (4, 0)

	while True:
		a, w = bot.plant()
		bot.update(a, w, dt)
		bot.pid(dt)
		bot.rrt()
		bot.print()
		time.sleep(0.2)

if __name__ == '__main__':
	main()