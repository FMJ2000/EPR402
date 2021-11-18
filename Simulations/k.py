import numpy as np
import matplotlib.pyplot as plt

# distance
da = 0.008
dk = 0.4	
dt = np.linspace(0.0, 0.9, 100)
dx = (dk*dt) / (da + dt)

# angle
ra = 0.4
rk = np.pi
rt = np.linspace(0.0, np.pi, 100)
rx = (rk*rt) / (ra + rt)

fig, axes = plt.subplots(1, 2, figsize=(10,5))
axes[0].plot(dt, dx)
axes[0].grid()
axes[0].set_title('desired velocity given error distance')
axes[0].set_ylabel('velocity')
axes[0].set_xlabel('error distance')

axes[1].plot(rt, rx)
axes[1].grid()
axes[1].set_title('desired angular rate given error angle')
axes[1].set_ylabel('angular rate')
axes[1].set_xlabel('error angle')

plt.show()
