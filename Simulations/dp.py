import numpy as np
np.set_printoptions(suppress=True, precision=3)

P = np.array([
	[3, 1, -3],
	[0.5, 1, 0.5],
	[-3, 1, 3]
])

d = np.array([
	[0.6, 0.6, 0.6],
	[0.6, 0.3, 0.3],
	[0.6, 0.6, 0.3],
	[0.3, 0.6, 0.3],
	[0.3, 0.3, 0.6],
	[0.3, 0.3, 0.3],
	[0.6, 0.6, 0.13]
])

for di in d:
	mu = np.matmul(P, di.T)
	mu = np.concatenate((mu, [1/np.sum(mu)]))
	#mu = mu / np.linalg.norm(mu)
	print(f'cmu: {mu}, mu: {(np.where(mu == np.max(mu))[0][0])}, sigma: {(1 / (1 + np.max(mu))):.2f}')
	
