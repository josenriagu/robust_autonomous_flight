import numpy as np

class Aircraft:
    def __init__(self, state):
        self.state = state.copy()
        self.time = 0.0

    def dynamics(self, state, acceleration):
        px, py, vx, vy = state
        ax, ay = acceleration
        return np.array([vx, vy, ax, ay])
    
    def step(self, acceleration, dt):
        state_derivative = self.dynamics(self.state, acceleration)
        self.state = self.state + state_derivative * dt
        self.time += dt
