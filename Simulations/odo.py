import matplotlib.pyplot as plt
import numpy as np

t = np.linspace(-np.pi/2, np.pi/2, 100)
x = 1.6*t**2
for i in range(len(x)):
	if x[i] > 0.16:
		x[i] = 0.16
y = 0.16 - x

a = 1 / 1.5961632439130233
print(a)
s = []
for i in range(20):
	s.append(a / (i+1)**2)
print(s)
print(sum(s))

st = np.linspace(1, 20, 20)
plt.figure()
plt.plot(st, s)

plt.figure()
plt.plot(t, x)
plt.plot(t, y)
plt.plot(t, x+y)
plt.show()

