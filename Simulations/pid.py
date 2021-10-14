import math, random, time

WHEEL_DIAMETER = 22
WHEEL_RADIUS = 15

class Node():
	def __init__(self, pos, parent):
		self.pos = pos
		self.edges = [parent]
		parent.edges.append(self)

class Bot():
	def __init__(self):
		self.max_vel = 1.0				# maximum m/s at full pwm
		self.vel = [0.0, 0.0]
		self.pos = [0.0, 0.0]
		self.rot = 0.0
		self.rot_sigma = 0.05
		self.Q = 0.0005
		self.R = 0.0005
		self.a = None
		self.w = None

		self.r_pos = [0.0, 0.0]
		self.e_rot_s = 0.0
		self.e_rot = 0.0
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
		self.rot += w[2] * dt

		'''
		e_mu = self.rot + w[2] * dt
		e_sigma = self.rot_sigma + self.Q

		# measurement and Kalman gain
		z = math.atan(math.sqrt(a[0]**2 + a[1]**2) / a[2])
		K = e_sigma / (e_sigma + self.R)
		self.rot = e_mu + K * (z - e_mu)
		self.rot_sigma = e_sigma - K * e_sigma

		#print(f'e_mu: {e_mu:.2f}, e_sigma: {e_sigma:.2f}, z: {z:.2f}, K: {K:.2f}')
		'''
		# update position based on rotation
		self.vel[0] += a[0] * dt * math.cos(self.rot) - a[1] * dt * math.sin(self.rot)
		self.vel[1] += + a[0] * dt * math.sin(self.rot) + a[1] * dt * math.cos(self.rot)
		self.pos[0] += self.vel[0] * dt * math.cos(self.rot) - self.vel[1] * dt * math.sin(self.rot)
		self.pos[1] += self.vel[0] * dt * math.sin(self.rot) + self.vel[1] * dt * math.cos(self.rot)

	def pid(self, dt):
		# desired and error orientation
		r_rot = math.atan2(self.r_pos[1] - self.pos[1], self.r_pos[0] - self.pos[0])
		self.e_rot = r_rot - self.rot
		self.e_rot_s += self.e_rot * dt
		#print(f'rot: {self.rot:.2f}, r_rot: {r_rot:.2f}, e_rot: {e_rot:.2f}')

		# obtain PWM from PI controller
		self.pwm[0] = 0.5 - self.K_p * self.e_rot - self.K_i * self.e_rot_s
		self.pwm[1] = 0.5 + self.K_p * self.e_rot + self.K_i * self.e_rot_s

	# simulate how movement will impact sensor readings
	def plant(self, dt):
		a = [random.gauss(0, self.R), random.gauss(0, self.R), random.gauss(1, self.R)]
		w = [random.gauss(0, self.R) for _ in range(3)]
		pwm = [self.pwm[0] + random.gauss(0, self.Q), self.pwm[1] + random.gauss(0, self.Q)]

		# arc given by difference in pwm for angle
		arc = [pwm[0]*self.max_vel*dt, pwm[1]*self.max_vel*dt]
		r_ex = 0
		if arc[0] > arc[1]:
			# left side drove faster than right - turning right
			r_ex = arc[1] * WHEEL_RADIUS / (arc[0] - arc[1])
			w[2] += arc[0] / (WHEEL_RADIUS + r_ex)
		elif arc[1] > arc[0]:
			# right side drove faster than left - turning left
			r_ex = arc[0] * WHEEL_RADIUS / (arc[1] - arc[0])
			w[2] += arc[1] / (WHEEL_RADIUS + r_ex)

		# difference in current and previous velocities for acceleration
		vel = [pwm[0] * self.max_vel * dt, pwm[1] * self.max_vel * dt]
		acc = [(vel[0] - self.vel[0]) / dt, (vel[1] - self.vel[1]) / dt]
		a[0] = 0.5 * (acc[0] + acc[1]) * math.cos(w[2])
		a[1] = 0.5 * (acc[0] + acc[1]) * math.sin(w[2])

		print(f'vel: {vel}')
		print(f'acc: {acc}')
		print(f'S: ({arc[0]:.2f}, {arc[1]:.2f}), r: {(WHEEL_RADIUS + r_ex):.2f}, angle: {w[2]:.2f}')
		
		return a, w

	def rrt(self):
		# test if new position required
		if math.sqrt((self.r_pos[0] - self.pos[0])**2 + (self.r_pos[1] - self.pos[1])**2) < self.min_dist:
			self.r_pos = (random.randrange(int(self.pos[0] - self.max_dist), int(self.pos[0] + self.max_dist)), random.randrange(int(self.pos[1] - self.max_dist), int(self.pos[1] + self.max_dist)))

	def print(self):
		print(f'({self.pos[0]:.2f}, {self.pos[1]:.2f}, {(self.rot * 180 / math.pi):.2f}) -> ({self.r_pos[0]:.2f}, {self.r_pos[1]:.2f}, {(self.e_rot * 180 / math.pi):.2f}) | pwm: ({self.pwm[0]:.2f}, {self.pwm[1]:.2f}) | a: ({self.a[0]:.2f}, {self.a[1]:.2f}, {self.a[2]:.2f}), w: ({self.w[0]:.2f}, {self.w[1]:.2f}, {self.w[2]:.2f})')

def main():
	random.seed(time.time())
	bot = Bot()
	dt = 0.2
	bot.r_pos = (4, 0)

	while True:
		a, w = bot.plant(dt)
		bot.update(a, w, dt)
		bot.pid(dt)
		#bot.rrt()
		bot.print()
		input()
if __name__ == '__main__':
	main()