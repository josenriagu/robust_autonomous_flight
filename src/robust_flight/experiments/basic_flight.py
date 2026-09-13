import numpy as np
import matplotlib.pyplot as plt
from robust_flight.dynamics.aircraft import Aircraft
from robust_flight.simulation.engine import simulate

state = np.array([0.0, 0.0, 10.0, 0.0])

aircraft = Aircraft(state)

acceleration = np.array([2.0, 1.0])

history = simulate(aircraft, acceleration, 10.0, 0.1)
print(history.shape)

x = history[:, 0]
y = history[:, 1]

plt.plot(x, y)
plt.xlabel('x position (m)')
plt.ylabel('y position (m)')
plt.title('Aircraft Trajectory')
plt.show()