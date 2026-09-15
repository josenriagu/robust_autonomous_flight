import numpy as np

def simulate(aircraft, control_function, duration, dt):
    num_steps = int(duration / dt)
    history = [aircraft.state.copy()]
    
    for _ in range(num_steps):
        control = control_function(aircraft.state)
        aircraft.step(control, dt)
        history.append(aircraft.state.copy())
        
    return np.array(history)
