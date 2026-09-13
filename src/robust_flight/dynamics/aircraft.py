import numpy as np
import matplotlib.pyplot as plt


state = np.array([0.0, 0.0, 10.0, 0.0])

class Aircraft:
    def __init__(self, state):
        self.state = state.copy()
        self.time = 0.0
    def dynamics(self, state, acceleration):
        px, py, vx, vy = state
        ax, ay = acceleration
        return np.array([vx, vy, ax, ay])
    def step(self, acceleration, dt):
        self.derivative = self.dynamics(self.state, acceleration)
        self.state = self.state + self.derivative * dt
        self.time += dt
        
def simulate(aircraft, acceleration,duration, dt):
    num_steps = int(duration / dt)
    history = [aircraft.state.copy()]
    
    for _ in range(num_steps):
        aircraft.step(acceleration, dt)
        history.append(aircraft.state.copy())
        
    return np.array(history)

aircraft = Aircraft(state)

print(aircraft.state)
print(aircraft.time)

acceleration = np.array([2.0, 1.0])

# aircraft.step(acceleration, 0.1)

# print(aircraft.state)
# print(aircraft.time)

history = simulate(aircraft, acceleration, 10.0, 0.1)
print(history.shape)

x = history[:, 0]
y = history[:, 1]

plt.plot(x, y)
plt.xlabel('x position (m)')
plt.ylabel('y position (m)')
plt.title('Aircraft Trajectory')
plt.show()


