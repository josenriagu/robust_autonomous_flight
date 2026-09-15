import numpy as np

class Aircraft:
    def __init__(self, state):
        self.state = state.copy()
        self.time = 0.0

    def dynamics(self, state, control):
        px, py, v, psi = state
        a, r = control
        return np.array([v * np.cos(psi), v * np.sin(psi), a, r])
    
    def step(self, control, dt):
        state_derivative = self.dynamics(self.state, control)
        self.state = self.state + state_derivative * dt
        self.time += dt
