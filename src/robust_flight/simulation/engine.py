import numpy as np

def simulate(aircraft, acceleration,duration, dt):
    num_steps = int(duration / dt)
    history = [aircraft.state.copy()]
    
    for _ in range(num_steps):
        aircraft.step(acceleration, dt)
        history.append(aircraft.state.copy())
        
    return np.array(history)
