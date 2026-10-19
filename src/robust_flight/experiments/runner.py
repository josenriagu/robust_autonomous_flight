from dataclasses import dataclass

import numpy as np

from robust_flight.control.heading import HeadingController
from robust_flight.control.speed import SpeedController
from robust_flight.dynamics.actuator import Actuator
from robust_flight.dynamics.aircraft import Aircraft
from robust_flight.mission.mission import Mission
from robust_flight.simulation.engine import SimulationResult, simulate


@dataclass(frozen=True)
class ExperimentConfig:
    """Parameters for a single closed-loop flight experiment.

    The defaults describe the baseline scenario. A sweep varies one
    parameter with dataclasses.replace, so every other parameter stays
    at its baseline value.
    """

    # simulation parameters
    duration: float = 10.0
    dt: float = 0.1

    # initial state [px, py, v, psi] and mission waypoints
    initial_state: tuple = (0.0, 0.0, 10.0, 0.0)
    waypoints: tuple = ((50.0, 30.0),)

    # guidance parameters
    max_speed: float = 20.0
    max_deceleration: float = 2.0

    # controller gains
    heading_kp: float = 2.0
    speed_kp: float = 1.0

    # actuator limits
    min_acceleration: float = -2.0
    max_acceleration: float = 3.0
    max_turn_rate: float = np.pi / 3


@dataclass
class ExperimentResult:
    config: ExperimentConfig
    simulation: SimulationResult
    history: np.ndarray
    desired_speeds: np.ndarray
    speed_errors: np.ndarray
    final_position: np.ndarray
    final_distance: float
    final_speed: float
    final_desired_speed: float
    minimum_distance: float
    cross_track_errors: np.ndarray
    cross_track_rmse: float
    max_cross_track_error: float
    speed_rmse: float
    max_speed_error: float
    min_observed_acceleration: float
    max_observed_acceleration: float
    final_time: float
    mission_complete: bool
    acceleration_saturation_rate: float
    turn_rate_saturation_rate: float
    max_acceleration_intervention: float
    mean_acceleration_intervention: float
    max_turn_rate_intervention: float
    mean_turn_rate_intervention: float
    acceleration_effort: float
    turn_rate_effort: float


def run_experiment(config):
    aircraft = Aircraft(np.array(config.initial_state))
    mission = Mission(config.waypoints)
    heading_controller = HeadingController(kp=config.heading_kp)
    speed_controller = SpeedController(kp=config.speed_kp)
    actuator = Actuator(
        min_acceleration=config.min_acceleration,
        max_acceleration=config.max_acceleration,
        max_turn_rate=config.max_turn_rate,
    )

    def control_function(state):
        waypoint = mission.current_waypoint()

        if waypoint is None:
            return np.array([0.0, 0.0])

        desired_heading = mission.waypoints.desired_heading(state[:2], waypoint)
        heading_error = mission.waypoints.heading_error(state[3], desired_heading)
        desired_speed = mission.waypoints.desired_speed(
            state[:2],
            waypoint,
            config.max_speed,
            config.max_deceleration,
        )

        acceleration = speed_controller.control(desired_speed - state[2])
        turn_rate = heading_controller.control(heading_error)
        return np.array([acceleration, turn_rate])

    def termination_condition(state):
        mission.update(state[:2])
        return mission.is_mission_complete()

    simulation = simulate(
        aircraft,
        control_function,
        config.duration,
        config.dt,
        termination_condition,
        actuator=actuator,
    )
    history = simulation.states
    dt = config.dt

    # metrics are measured against the final waypoint, which is only
    # exact for single-waypoint missions
    waypoint = mission.waypoints.points[-1]

    # calculate speed tracking
    desired_speeds = np.array(
        [
            mission.waypoints.desired_speed(
                position,
                waypoint,
                config.max_speed,
                config.max_deceleration,
            )
            for position in history[:, :2]
        ]
    )
    speed_errors = desired_speeds - history[:, 2]
    observed_accelerations = np.diff(history[:, 2]) / dt

    # calculate distance to waypoint
    final_position = history[-1, :2]
    final_distance = mission.waypoints.distance_to_waypoint(final_position, waypoint)
    distances = np.linalg.norm(history[:, :2] - waypoint, axis=1)

    # calculate deviation from the straight path to the waypoint
    path_start = np.array(config.initial_state[:2])
    cross_track_errors = np.array(
        [
            mission.waypoints.cross_track_error(position, path_start, waypoint)
            for position in history[:, :2]
        ]
    )

    # calculate actuator intervention and control effort
    applied_controls = simulation.applied_controls
    control_difference = simulation.requested_controls - applied_controls
    acceleration_intervention = np.abs(control_difference[:, 0])
    turn_rate_intervention = np.abs(control_difference[:, 1])

    return ExperimentResult(
        config=config,
        simulation=simulation,
        history=history,
        desired_speeds=desired_speeds,
        speed_errors=speed_errors,
        final_position=final_position,
        final_distance=float(final_distance),
        final_speed=float(history[-1, 2]),
        final_desired_speed=float(desired_speeds[-1]),
        minimum_distance=float(np.min(distances)),
        cross_track_errors=cross_track_errors,
        cross_track_rmse=float(np.sqrt(np.mean(cross_track_errors**2))),
        max_cross_track_error=float(np.max(np.abs(cross_track_errors))),
        speed_rmse=float(np.sqrt(np.mean(speed_errors**2))),
        max_speed_error=float(np.max(np.abs(speed_errors))),
        min_observed_acceleration=float(np.min(observed_accelerations)),
        max_observed_acceleration=float(np.max(observed_accelerations)),
        final_time=float((len(history) - 1) * dt),
        mission_complete=mission.is_mission_complete(),
        acceleration_saturation_rate=float(np.mean(control_difference[:, 0] != 0.0)),
        turn_rate_saturation_rate=float(np.mean(control_difference[:, 1] != 0.0)),
        max_acceleration_intervention=float(np.max(acceleration_intervention)),
        mean_acceleration_intervention=float(np.mean(acceleration_intervention)),
        max_turn_rate_intervention=float(np.max(turn_rate_intervention)),
        mean_turn_rate_intervention=float(np.mean(turn_rate_intervention)),
        acceleration_effort=float(np.sum(applied_controls[:, 0] ** 2) * dt),
        turn_rate_effort=float(np.sum(applied_controls[:, 1] ** 2) * dt),
    )
