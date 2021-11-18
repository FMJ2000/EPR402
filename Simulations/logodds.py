import numpy as np
import matplotlib.pyplot as plt
np.set_printoptions(precision=2, suppress=True)

PH = 0.95
PL = 0.05
LH = np.log(PH / (1 - PH))
LL = np.log(PL / (1 - PL))

''''
p = np.linspace(0, 1, 100)
l = np.log(p / (1-p))
m = 1 - 2*np.abs(p - 0.5)
lm = np.log(PH/(1-PH)) + np.log(PL/(1-PL)) * np.abs(l)

plt.plot(p, l)
#plt.plot(p, lm)
plt.grid()
plt.title('Log-odds vs probability graph')
plt.xlabel(r'Probability $p(x)$')
plt.ylabel(r'Log-odds $l(x)$')
plt.show()
'''

def logodds_plot():
	p = np.mgrid[PL:PH:0.05, PL:PH:0.05]#np.array([np.linspace(PL, PH, 100), np.linspace(PL, PH, 100)])
	lm = np.log(p / (1-p))#np.array([np.log(p[0] / (1 - p[0])), np.log(p[1] / (1 - p[1]))])
	le = LL*np.abs(lm[0]) - lm[1]

	print(f'p: {p}')
	print(f'lm: {lm}')
	print(f'le: {le}')

	fig = plt.figure()
	ax = plt.axes(projection='3d')
	ax.scatter3D(lm[0], lm[1], le, c=le, cmap='Greens')
	ax.set_xlabel(r'cell $l(u)$')
	ax.set_ylabel(r'neighbour argmin $l(u)$')
	ax.set_zlabel(r'$l(e)$')
	ax.set_title('Edge detection in log-odds space')
	plt.show()

logodds_plot()