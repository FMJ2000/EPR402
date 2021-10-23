import numpy as np
np.set_printoptions(suppress=True, precision=3)

P = np.array([
	[3, -1, -1],
	[-1, 5, -1],
	[-1, -1, 3]
])

d = np.array([
	[0.6, 0.6, 0.6],
	[0.6, 0.3, 0.3],
	[0.6, 0.6, 0.3],
	[0.3, 0.6, 0.3],
	[0.3, 0.3, 0.6],
	[0.3, 0.3, 0.3]
])

for di in d:
	mu = np.matmul(P, di.T)
	mu = np.concatenate((mu, [2 / np.sum(np.power(mu, 2))]))
	#mu = mu / np.linalg.norm(mu)
	print(f'cmu: {mu}, mu: {(np.where(mu == np.max(mu))[0][0])}, sigma: {(1 / (1 + np.max(mu))):.2f}')
	
