import numpy as np, time
import matplotlib.pyplot as plt

def pseudo_uniform_good(mult=16807, mod=(2**31)-1, seed=123456789, size=1):
	U = np.zeros(size)
	x = (seed*mult+1) % mod
	U[0] = x / mod
	for i in range(1, size):
		x = (x*mult+1) % mod
		U[i] = x/mod
	return U

def pseudo_uniform(low=0, high=1, seed=123456789, size=1):
	return low + (high - low)*pseudo_uniform_good(seed=seed, size=size)

def pseudo_normal(mu=0.0, sigma=1.0, size = 1):
	t = time.perf_counter()
	seed1 = int(10**9*float(str(t-int(t))[0:]))
	U1 = pseudo_uniform(seed=seed1, size=size)
	t = time.perf_counter()
	seed2 = int(10**9*float(str(t-int(t))[0:]))
	U2 = pseudo_uniform(seed=seed2, size=size)
	
	Z0 = np.sqrt(-2*np.log(U1)) * np.cos(2*np.pi*U2)
	Z1 = np.sqrt(-2*np.log(U1)) * np.sin(2*np.pi*U2)
	Z0 = Z0 * sigma + mu
	return Z0
	
def pseudo_multinomial(mu=[0], sigma=[1], size=1):
	Z0 = pseudo_normal(mu=mu[0], sigma=sigma[0], size=int(size/len(mu)))
	for i in range(1, len(mu)):
		Z0 = np.concatenate((Z0, pseudo_normal(mu=mu[i], sigma=sigma[i], size=int(size/len(mu)))))
	return Z0

s = pseudo_multinomial(mu=[0, 10, 20, 30, 40, 50, 60, 70], sigma=[5, 2, 5, 2, 5, 2, 5, 2], size=10000)
print(s)
plt.hist(s, bins=40, edgecolor='k')
plt.xticks(fontsize=15)
plt.yticks(fontsize=15)
plt.show()

