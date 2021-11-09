import numpy as np
import matplotlib.pyplot as plt

a = 0.005
b = 0.001
k = 0.18
t = np.linspace(0.05, 0.9, 100)
x = k - a / t # - b / t**2

plt.plot(t, x)
plt.grid()
plt.show()
