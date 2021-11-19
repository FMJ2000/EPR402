import numpy as np
import matplotlib.pyplot as plt

# distance
da = 0.02
dk = 0.18	
dt = np.linspace(0.0, 0.9, 100)
dx = (dk*dt) / (da + dt)

# angle
ra = 0.8
rk = 0.15
rt = np.linspace(-np.pi, np.pi, 100)
rx = (rk*rt) / (ra + np.abs(rt))

dxx = (1 - rx**2 / rk**2) * (dk*0.2) / (da + 0.2)

fig, axes = plt.subplots(1, 2, figsize=(10,5))
axes[0].plot(rt, dxx)
axes[0].grid()
axes[0].set_title('desired velocity given error distance')
axes[0].set_ylabel('velocity')
axes[0].set_xlabel('error distance')

axes[1].plot(dt, dx)
axes[1].grid()
axes[1].set_title('desired angular rate given error angle')
axes[1].set_ylabel('angular rate')
axes[1].set_xlabel('error angle')

plt.show()
