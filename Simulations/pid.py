import math

class Bot():
	def __init__(self):
		self.vel = [0.0, 0.0]
		self.pos = [0.0, 0.0]
		self.rot = 0.0
		self.rot_sigma = 0.05
		self.q = 0.005
		self.r = 0.005

		self.r_pos = [20.0, 10.0]
		self.e_rot_s = 0.0
		self.K_p = 2
		self.K_i = 0.01
		self.pwm = [0.0, 0.0]

		self.max_dist = 3.0

	def update(self, a, w, dt):
		# kalman filter for orientation estimation
		# process
		e_mu = self.rot + w[2] * dt
		e_sigma = self.rot_sigma + self.q

		# measurement and Kalman gain
		z = math.atan(math.sqrt(a[0]**2 + a[1]**2) / a[2])
		K = e_sigma / (e_sigma + self.r)
		self.rot = e_mu + K * (z - e_mu)
		self.rot_sigma = e_sigma - K * e_sigma

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

		# obtain PWM from PI controller
		self.pwm[0] = 0.5 - self.K_p * e_rot - self.K_i * self.e_rot_s
		self.pwm[1] = 0.5 + self.K_p * e_rot + self.K_i * self.e_rot_s

	def rrt(self):
		# test if new position required


def main():
	bot = Bot()

if __name__ == '__main__':
	main()