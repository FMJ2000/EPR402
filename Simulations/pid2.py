import numpy as np, time

WHEEL_DIAMETER = 0.22
WHEEL_RADIUS = 0.15
VEL_MAX = 1.0
ERROR_MAX = 45.0 * np.pi / 180.0					# error angle to just turn in a circle instead of moving forward
IN_RANGE = 0.05														# bot close enough to desired position

class Bot:
	def __init__(self, f):
		# state
		self.dt = 1.0 / f
		self.counter = 0
		self.pos = np.array([0., 0., 0.])			# x, y, theta
		self.duty = np.array([0., 0.])				# left, right
		self.input = np.array([0., 0.])				# x, y
		self.acc = np.array([0., 0., 0.])			# x, y, z
		self.gyro = np.array([0., 0., 0.])		# roll, pitch, yaw

		# kalman filter
		self.sigma = np.diag([1., 1., 1.])
		self.Q = np.full(np.shape(self.sigma), 0.005)
		self.R = np.full(np.shape(self.sigma), 0.05)

		# estimation
		self.vel = np.array([0., 0.])
		self.pos_ = np.array([0., 0., 0.])
		self.acc_ = np.array([0., 0., 0.])
		self.gyro_ = np.array([0., 0., 0.])
		self.sigma_ = np.full(np.shape(self.sigma), 0.)

		# pid
		self.alpha = 0.1											# determines dampness of system speed
		self.beta = np.pi / 2									# determines dampness of system rotation
		self.gamma = 0.1											# maximum duty cycle may change per timestep
	
		# plant errors
		self.r_pos = np.array([0., 0., 0.])		# real position
		self.r_duty = np.array([0., 0.])			# real duty cycle
		self.r_acc = np.array([0., 0., 0.])		# real acceleration
		self.r_gyro = np.array([0., 0., 0.])	# real angular rate

	def process(self):
		self.counter += self.dt
		# estimate state
		v = [self.duty[0] * VEL_MAX, self.duty[1] * VEL_MAX]
		d_pos = np.array([0., 0., 0.])				# delta position
		self.pos_[2] = self.pos[2]
		if v[0] == v[1]:
			# straight ahead
			d_pos[0] = v[0] * self.dt * np.cos(self.pos_[2])
			d_pos[1] = v[0] * self.dt * np.sin(self.pos_[2])

		else:
			
			r = 0
			if v[0] > v[1]:
				# turning right
				r = v[1] / (v[0] - v[1]) * WHEEL_RADIUS
				d_pos[2] = -(v[0] * self.dt / (WHEEL_RADIUS + r))
			elif v[0] < v[1]:
				# turning left
				r = v[0] / (v[1] - v[0]) * WHEEL_RADIUS
				d_pos[2] = v[1] * self.dt / (WHEEL_RADIUS + r)
			d = 2 * (WHEEL_RADIUS + r) * np.sin(abs(d_pos[2]) / 2)
			#print(f'd": {(v[0] * self.dt):.3f} d: {d:.3f}, r: {r:.3f}, theta: {d_pos[2]:.3f}')
			self.pos_[2] = self.pos[2] + d_pos[2]
			d_pos[0] = d * np.cos(self.pos_[2])
			d_pos[1] = -d * np.sin(self.pos_[2])
		
		# estimate sensor measurements
		self.pos_[0] = self.pos[0] + d_pos[0]
		self.pos_[1] = self.pos[1] + d_pos[1]
		vel = np.array([d_pos[0] / self.dt, d_pos[1] / self.dt])
		#print(f'vel: {d_pos}')
		self.acc_ =  np.array([(vel[0] - self.vel[0]) / self.dt, (vel[1] - self.vel[1]) / self.dt, 1.0])
		self.gyro_ = np.array([0., 0., d_pos[2] / 2.0])
		self.vel = vel
		sig = np.array([
			[d_pos[0]**2, d_pos[0]*d_pos[1], d_pos[0]*d_pos[2]],
			[d_pos[1]*d_pos[0], d_pos[1]**2, d_pos[1]*d_pos[2]],
			[d_pos[2]*d_pos[0], d_pos[2]*d_pos[1], d_pos[2]**2],
		])
		self.sigma_ = self.sigma + np.absolute(sig) + self.Q

	def measure(self, acc, gyro):
		self.acc = np.array(acc)
		self.gyro = np.array(gyro)

		d_acc = self.acc - self.acc_
		d_gyro = self.gyro - self.gyro_

		self.pos = self.pos_

	def pid(self):
		# obtain error angle and distance
		theta = 0
		if self.input[0] == self.pos[0]:
			theta = np.sign(self.pos[1] - self.input[1]) * np.pi / 2
		else:
			theta = (np.arctan(-(self.input[1] - self.pos[1]) / (self.input[0] - self.pos[0])))
		if (self.input[0] - self.pos[0]) < 0:
			theta = np.pi + theta
		dist = np.sqrt((self.input[0] - self.pos[0])**2 + (self.input[1] - self.pos[1])**2)

		e_pos = np.array([0., 0., 0.])
		e_pos[2] = theta - self.pos[2]
		max_speed = 0
		max_turn = abs(e_pos[2]) / (self.beta + abs(e_pos[2]))
		new_duty = np.array([0., 0.])

		#print(f'error: {e_pos}')

		if abs(e_pos[0]) > ERROR_MAX:
			# turn without moving
			new_duty[0] = np.sign(e_pos[2]) * max_turn
			new_duty[1] = np.sign(e_pos[2]) * -max_turn
		else:
			# turn and move
			max_speed = dist / (self.alpha + dist)
			new_duty[0] = max_speed / 2.0 + np.sign(e_pos[2]) * -max_turn
			new_duty[1] = max_speed / 2.0 + np.sign(e_pos[2]) * max_turn

		for i in range(2):
			if new_duty[i] < self.duty[i] - self.gamma:
				new_duty[i] = self.duty[i] - self.gamma
			elif new_duty[i] > self.duty[i] + self.gamma:
				new_duty[i] = self.duty[i] + self.gamma
			self.duty[i] = new_duty[i]

	def plant(self):
		pass

	def test_pos(self):
		d = np.sqrt((self.input[0] - self.pos[0])**2 + (self.input[1] - self.pos[1])**2)
		return (d <= IN_RANGE)

	def print_est(self):
		print(f'process: pos_: {self.pos_} | acc_: {self.acc_}, gyro_: {self.gyro_}')#, sigma_: {self.sigma_}')

	def print_real(self):
		print(f'measure: pos: {self.pos} | acc: {self.acc}, gyro: {self.gyro}')

	def print_pid(self):
		print(f't: {self.counter:.2f} pid: {self.pos} -> {self.input} | pwm: {self.duty} | acc_: {self.acc_}, gyro_: {self.gyro_}')

def main():
	np.set_printoptions(precision=3, suppress=True)
	bot = Bot(30.0)
	while True:
		print('Bot position (x,y): ', end='')
		bot.input = [float(i) for i in input().split(',')]
		while not bot.test_pos():
			bot.process()
			#bot.print_est()
			#print('acc[3], gyro[3]: ', end='')
			#sensor = [float(i) for i in input().split(',')]
			sensor = np.full((6), 0)
			bot.measure(sensor[:3], sensor[3:])
			#bot.print_real()
			bot.pid()
			bot.print_pid()
			time.sleep(1.0/30.0)

if __name__ == '__main__':
	main()
