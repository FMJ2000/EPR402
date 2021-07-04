from scipy.integrate import quad
import numpy as np
import matplotlib.pyplot as plt

x = np.linspace(-4, 4, num=100)
constant = 1.0 / np.sqrt(2*np.pi)
y = constant * np.exp((-x**2) / 2.0)
fig, ax = plt.subplots(figsize=(10, 5))
ax.plot(x, y)
ax.set_ylim(0);
ax.set_title('Normal Distribution', size=20)
ax.set_ylabel('Probability Density', size=20)
plt.show()
