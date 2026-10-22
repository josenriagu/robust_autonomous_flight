from dataclasses import dataclass

import numpy as np

from robust_flight.control.heading import HeadingController
from robust_flight.control.speed import SpeedController
from robust_flight.dynamics.actuator import Actuator
from robust_flight.dynamics.aircraft import Aircraft
from robust_flight.estimation.ekf import EKF
from robust_flight.mission.mission import Mission
from robust_flight.sensors.gps import GPS
from robust_flight.sensors.imu import IMU
from robust_flight.simulation.engine import SimulationResult, simulate


@dataclass(frozen=True)
class ExperimentConfig:
    """Parameters for a single closed-loop flight experiment.

    The defaults describe the baseline scenario. A sweep varies one
    parameter with dataclasses.replace, so every other parameter stays
    at its baseline value.
    """

    # simulation parameters
    duration: float = 60.0
    dt: float = 0.01

    # initial state [px, py, v, psi] and a 50 m square mission with
    # 90 degree turns that returns to the start
    initial_state: tuple = (0.0, 0.0, 10.0, 0.0)
    waypoints: tuple = ((50.0, 0.0), (50.0, 50.0), (0.0, 50.0), (0.0, 0.0))

    # a waypoint counts as reached within this distance, in m
    arrival_radius: float = 1.0

    # guidance parameters; braking is planned at 75% of the actuator's
    # deceleration limit so the speed controller has margin to catch up
    max_speed: float = 20.0
    max_deceleration: float = 1.5

    # controller gains
    heading_kp: float = 2.0
    speed_kp: float = 1.0

    # actuator limits
    min_acceleration: float = -2.0
    max_acceleration: float = 3.0
    max_turn_rate: float = np.pi / 3

    # random seed; each sensor draws from its own stream split from it
    seed: int = 0

    # GPS: position and velocity at a fixed rate, in m and m/s
    gps_rate: float = 5.0
    gps_position_noise: float = 1.0
    gps_velocity_noise: float = 0.1
    gps_position_bias: tuple = (0.0, 0.0)
    gps_dropout_probability: float = 0.0

    # IMU: acceleration and yaw rate every step, in m/s² and rad/s
    imu_acceleration_noise: float = 0.05
    imu_yaw_rate_noise: float = 0.01
    imu_acceleration_bias: float = 0.0
    imu_yaw_rate_bias: float = 0.0

    # EKF starting uncertainty, in m, m/s and rad; its starting estimate is
    # the initial state, and it assumes the sensor noise levels above
    ekf_initial_position_std: float = 1.0
    ekf_initial_speed_std: float = 0.5
    ekf_initial_heading_std: float = np.radians(5.0)

    # the state the controller and the mission act on: "true" for the true
    # state, "gps" for the latest GPS sample, held until the next one, or
    # "ekf" for the EKF estimate
    feedback: str = "true"


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
    max_overspeed: float
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
    gps_times: np.ndarray
    gps_measurements: np.ndarray
    imu_measurements: np.ndarray
    gps_position_rmse: float
    gps_velocity_rmse: float
    feedback_states: np.ndarray
    feedback_position_rmse: float
    feedback_speed_rmse: float
    feedback_heading_rmse: float
    ekf_states: np.ndarray
    ekf_covariances: np.ndarray
    ekf_position_rmse: float
    ekf_speed_rmse: float
    ekf_heading_rmse: float
    ekf_nees: float


def run_experiment(config):
    if config.feedback not in ("true", "gps", "ekf"):
        raise ValueError(f"unknown feedback {config.feedback!r}")

    aircraft = Aircraft(np.array(config.initial_state))
    mission = Mission(config.waypoints, arrival_radius=config.arrival_radius)
    heading_controller = HeadingController(kp=config.heading_kp)
    speed_controller = SpeedController(kp=config.speed_kp)
    actuator = Actuator(
        min_acceleration=config.min_acceleration,
        max_acceleration=config.max_acceleration,
        max_turn_rate=config.max_turn_rate,
    )

    gps_rng, imu_rng = (
        np.random.default_rng(seed)
        for seed in np.random.SeedSequence(config.seed).spawn(2)
    )
    gps = GPS(
        gps_rng,
        rate=config.gps_rate,
        position_noise=config.gps_position_noise,
        velocity_noise=config.gps_velocity_noise,
        position_bias=config.gps_position_bias,
        dropout_probability=config.gps_dropout_probability,
    )
    imu = IMU(
        imu_rng,
        acceleration_noise=config.imu_acceleration_noise,
        yaw_rate_noise=config.imu_yaw_rate_noise,
        acceleration_bias=config.imu_acceleration_bias,
        yaw_rate_bias=config.imu_yaw_rate_bias,
    )

    gps_steps = []
    gps_times = []
    gps_measurements = []
    imu_measurements = []

    # the latest GPS sample as a state; before the first sample arrives the
    # aircraft knows only its initial state
    gps_state = np.array(config.initial_state, dtype=float)

    # the EKF runs in every mode, so its accuracy can be compared with the
    # other feedback options on the same flight
    ekf = EKF(
        config.initial_state,
        np.diag(
            [
                config.ekf_initial_position_std**2,
                config.ekf_initial_position_std**2,
                config.ekf_initial_speed_std**2,
                config.ekf_initial_heading_std**2,
            ]
        ),
        acceleration_noise=config.imu_acceleration_noise,
        yaw_rate_noise=config.imu_yaw_rate_noise,
        position_noise=config.gps_position_noise,
        velocity_noise=config.gps_velocity_noise,
    )
    ekf_states = [ekf.state.copy()]
    ekf_covariances = [ekf.covariance.copy()]

    def record_measurements(time, state, applied_control):
        nonlocal gps_state
        imu_measurement = imu.measure(applied_control)
        imu_measurements.append(imu_measurement)
        ekf.predict(imu_measurement, config.dt)

        gps_measurement = gps.measure(time, state)
        if gps_measurement is not None:
            gps_steps.append(len(imu_measurements))
            gps_times.append(time)
            gps_measurements.append(gps_measurement)
            gps_state = state_from_gps(gps_measurement)
            ekf.update(gps_measurement)

        ekf_states.append(ekf.state.copy())
        ekf_covariances.append(ekf.covariance.copy())

    def believed_state(true_state):
        if config.feedback == "gps":
            return gps_state
        if config.feedback == "ekf":
            return ekf.state.copy()
        return true_state

    # index of the waypoint the controller steered towards at each step, and
    # the state it steered on
    active_waypoint_indices = []
    feedback_states = []

    def control_function(true_state):
        state = believed_state(true_state)
        feedback_states.append(state.copy())
        active_waypoint_indices.append(mission.current_waypoint_index)
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

    # waypoints are reached by the position the aircraft believes it is at
    def termination_condition(true_state):
        mission.update(believed_state(true_state)[:2])
        return mission.is_mission_complete()

    simulation = simulate(
        aircraft,
        control_function,
        config.duration,
        config.dt,
        termination_condition,
        actuator=actuator,
        step_callback=record_measurements,
    )
    history = simulation.states
    dt = config.dt

    # each leg runs from the previous waypoint, or the initial position,
    # to the active waypoint; the final state stays on the last leg
    points = mission.waypoints.points
    leg_starts = np.vstack([config.initial_state[:2], points[:-1]])
    leg_indices = active_waypoint_indices + active_waypoint_indices[-1:]
    leg_start_points = leg_starts[leg_indices]
    leg_end_points = points[leg_indices]
    waypoint = points[-1]

    # calculate speed tracking
    desired_speeds = np.array(
        [
            mission.waypoints.desired_speed(
                position,
                leg_end,
                config.max_speed,
                config.max_deceleration,
            )
            for position, leg_end in zip(history[:, :2], leg_end_points)
        ]
    )
    speed_errors = desired_speeds - history[:, 2]

    # overspeed is flying faster than the desired speed; the largest
    # under-speed is set by the initial state rather than the controller
    max_overspeed = max(0.0, -np.min(speed_errors))
    observed_accelerations = np.diff(history[:, 2]) / dt

    # calculate distance to waypoint
    final_position = history[-1, :2]
    final_distance = mission.waypoints.distance_to_waypoint(final_position, waypoint)
    distances = np.linalg.norm(history[:, :2] - waypoint, axis=1)

    # calculate deviation from the straight path of the active leg
    cross_track_errors = np.array(
        [
            mission.waypoints.cross_track_error(position, leg_start, leg_end)
            for position, leg_start, leg_end in zip(
                history[:, :2], leg_start_points, leg_end_points
            )
        ]
    )

    # calculate actuator intervention and control effort
    applied_controls = simulation.applied_controls
    control_difference = simulation.requested_controls - applied_controls
    acceleration_intervention = np.abs(control_difference[:, 0])
    turn_rate_intervention = np.abs(control_difference[:, 1])

    # calculate GPS error against the true state at each sample; a step's
    # callback sees the state recorded at the same index in the history
    gps_measurements = np.array(gps_measurements).reshape(-1, 4)
    true_states = history[gps_steps]
    true_velocities = np.column_stack(
        [
            true_states[:, 2] * np.cos(true_states[:, 3]),
            true_states[:, 2] * np.sin(true_states[:, 3]),
        ]
    )
    gps_position_errors = gps_measurements[:, :2] - true_states[:, :2]
    gps_velocity_errors = gps_measurements[:, 2:] - true_velocities

    # calculate the error in the state the controller steered on; step k
    # steered on its belief about the state recorded at index k
    feedback_states = np.array(feedback_states)
    feedback_errors = feedback_states - history[:-1]
    heading_errors = (feedback_errors[:, 3] + np.pi) % (2 * np.pi) - np.pi

    # calculate EKF error against the true state at every recorded index, and
    # its NEES, which averages 4 when the reported uncertainty is honest
    ekf_states = np.array(ekf_states)
    ekf_covariances = np.array(ekf_covariances)
    ekf_errors = ekf_states - history
    ekf_errors[:, 3] = (ekf_errors[:, 3] + np.pi) % (2 * np.pi) - np.pi
    weighted_errors = np.linalg.solve(ekf_covariances, ekf_errors[:, :, None])
    ekf_nees = np.einsum("ki,ki->k", ekf_errors, weighted_errors[:, :, 0])

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
        max_overspeed=float(max_overspeed),
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
        gps_times=np.array(gps_times),
        gps_measurements=gps_measurements,
        imu_measurements=np.array(imu_measurements),
        gps_position_rmse=rms_norm(gps_position_errors),
        gps_velocity_rmse=rms_norm(gps_velocity_errors),
        feedback_states=feedback_states,
        feedback_position_rmse=rms_norm(feedback_errors[:, :2]),
        feedback_speed_rmse=float(np.sqrt(np.mean(feedback_errors[:, 2] ** 2))),
        feedback_heading_rmse=float(np.sqrt(np.mean(heading_errors**2))),
        ekf_states=ekf_states,
        ekf_covariances=ekf_covariances,
        ekf_position_rmse=rms_norm(ekf_errors[:, :2]),
        ekf_speed_rmse=float(np.sqrt(np.mean(ekf_errors[:, 2] ** 2))),
        ekf_heading_rmse=float(np.sqrt(np.mean(ekf_errors[:, 3] ** 2))),
        ekf_nees=float(np.mean(ekf_nees[1:])),
    )


def state_from_gps(measurement):
    # speed and heading follow from the measured velocity vector
    px, py, vx, vy = measurement
    return np.array([px, py, np.hypot(vx, vy), np.arctan2(vy, vx)])


def rms_norm(errors):
    # root mean square of the error vector's length; nan when there are none
    if len(errors) == 0:
        return float("nan")
    return float(np.sqrt(np.mean(np.sum(errors**2, axis=1))))
