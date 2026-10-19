from dataclasses import dataclass

import numpy as np


def simulate(
    aircraft,
    control_function,
    duration,
    dt,
    termination_condition=None,
    actuator=None,
    step_callback=None,
):
    num_steps = int(duration / dt)
    history = [aircraft.state.copy()]
    requested_control_history = []
    applied_control_history = []

    for _ in range(num_steps):
        requested_control = control_function(aircraft.state)

        if actuator is not None:
            applied_control = actuator.apply(requested_control)
        else:
            applied_control = requested_control

        requested_control_history.append(requested_control.copy())
        applied_control_history.append(applied_control.copy())

        aircraft.step(applied_control, dt)
        history.append(aircraft.state.copy())

        # lets observers such as sensors see each step without the engine
        # knowing what they are
        if step_callback is not None:
            step_callback(aircraft.time, aircraft.state.copy(), applied_control.copy())

        if termination_condition is not None and termination_condition(aircraft.state):
            break
    return SimulationResult(
        states=np.array(history),
        requested_controls=np.array(requested_control_history),
        applied_controls=np.array(applied_control_history),
    )


@dataclass
class SimulationResult:
    states: np.ndarray
    requested_controls: np.ndarray
    applied_controls: np.ndarray
