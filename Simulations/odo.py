import matplotlib.pyplot as plt
import numpy as np

t = np.linspace(-np.pi/2, np.pi/2, 100)
x = 0.8*t**2
for i in range(len(x)):
	if x[i] > 0.16:
		x[i] = 0.16
y = 0.16 - x

plt.plot(t, x)
plt.plot(t, y)
plt.show()

