import numpy as np

from robust_flight.dynamics.actuator import Actuator
from robust_flight.dynamics.aircraft import Aircraft
from robust_flight.simulation.engine import simulate


def aircraft():
    return Aircraft(np.array([0.0, 0.0, 10.0, 0.0]))


def test_controller_is_called_once_per_step():
    calls = []

    def control_function(state):
        calls.append(state.copy())
        return np.array([0.0, 0.0])

    result = simulate(aircraft(), control_function, 1.0, 0.1)

    assert len(calls) == 10
    assert len(result.states) == 11


def test_requested_control_is_applied_without_an_actuator():
    result = simulate(aircraft(), lambda state: np.array([5.0, 0.1]), 1.0, 0.1)

    assert np.array_equal(result.requested_controls, result.applied_controls)


def test_actuator_clips_the_applied_control():
    actuator = Actuator(min_acceleration=-2.0, max_acceleration=3.0, max_turn_rate=1.0)
    result = simulate(
        aircraft(), lambda state: np.array([5.0, -4.0]), 1.0, 0.1, actuator=actuator
    )

    assert np.all(result.requested_controls == [5.0, -4.0])
    assert np.all(result.applied_controls == [3.0, -1.0])


def test_step_callback_sees_each_recorded_state_and_the_applied_control():
    seen = []
    actuator = Actuator(min_acceleration=-2.0, max_acceleration=3.0, max_turn_rate=1.0)
    result = simulate(
        aircraft(),
        lambda state: np.array([5.0, 0.0]),
        1.0,
        0.1,
        actuator=actuator,
        step_callback=lambda time, state, control: seen.append((time, state, control)),
    )

    assert len(seen) == 10
    assert np.allclose([time for time, _, _ in seen], np.arange(1, 11) * 0.1)
    for step, (_, state, control) in enumerate(seen):
        assert np.array_equal(state, result.states[step + 1])
        assert np.array_equal(control, [3.0, 0.0])


def test_termination_condition_stops_the_run_early():
    result = simulate(
        aircraft(),
        lambda state: np.array([0.0, 0.0]),
        10.0,
        0.1,
        termination_condition=lambda state: state[0] >= 5.0,
    )

    # at 10 m/s the aircraft passes 5 m after 5 steps
    assert len(result.states) == 6
