from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np

from robust_flight.control.heading import HeadingController
from robust_flight.control.speed import SpeedController
from robust_flight.dynamics.actuator import Actuator
from robust_flight.dynamics.aircraft import Aircraft
from robust_flight.mission.mission import Mission
from robust_flight.simulation.engine import simulate


@dataclass
class ExperimentResult:
    simulation: object
    history: np.ndarray
    desired_speeds: np.ndarray
    dt: float
    speed_kp: float
    final_position: np.ndarray
    final_distance: float
    final_speed: float
    final_desired_speed: float
    minimum_distance: float
    speed_rmse: float
    min_acceleration: float
    max_acceleration: float
    final_time: float
    acceleration_saturation_rate: float
    turn_rate_saturation_rate: float
    max_acceleration_intervention: float
    mean_acceleration_intervention: float
    max_turn_rate_intervention: float
    mean_turn_rate_intervention: float

def run_experiment(speed_kp):
    # simulation parameters
    duration = 10.0
    dt = 0.1

    # guidance parameters
    max_speed = 20.0
    max_deceleration = 2.0

    # controller gains
    heading_kp = 2.0

    # initial state
    state = np.array([0.0, 0.0, 10.0, 0.0])

    # create an aircraft instance from the class
    aircraft = Aircraft(state)

    # define the waypoint and create a Mission instance
    waypoint = np.array([50.0, 30.0])
    mission = Mission([waypoint])

    # create the heading and speed controllers
    heading_controller = HeadingController(kp=heading_kp)
    speed_controller = SpeedController(kp=speed_kp)

    # create the actuator model
    actuator = Actuator(
        min_acceleration=-2.0,
        max_acceleration=3.0,
        max_turn_rate=np.pi / 3,
    )

    def control_function(state):
        waypoint = mission.current_waypoint()

        if waypoint is None:
            return np.array([0.0, 0.0])

        desired_heading = mission.waypoints.desired_heading(
            state[:2],
            waypoint,
        )

        heading_error = mission.waypoints.heading_error(
            state[3],
            desired_heading,
        )

        desired_speed = mission.waypoints.desired_speed(
            state[:2],
            waypoint,
            max_speed,
            max_deceleration,
        )

        speed_error = desired_speed - state[2]

        acceleration = speed_controller.control(speed_error)
        heading_control = heading_controller.control(heading_error)

        return np.array([acceleration, heading_control])

    def termination_condition(state):
        mission.update(state[:2])
        return mission.is_mission_complete()

    # run simulation
    result = simulate(
        aircraft,
        control_function,
        duration,
        dt,
        termination_condition,
        actuator=actuator,
    )

    history = result.states

    # calculate final-state metrics
    final_position = history[-1, :2]

    final_distance = mission.waypoints.distance_to_waypoint(
        final_position,
        waypoint,
    )

    final_speed = history[-1, 2]

    final_desired_speed = mission.waypoints.desired_speed(
        final_position,
        waypoint,
        max_speed,
        max_deceleration,
    )

    # calculate distance to waypoint throughout the flight
    distances = np.linalg.norm(
        history[:, :2] - waypoint,
        axis=1,
    )

    minimum_distance = np.min(distances)

    # calculate desired speeds throughout the flight
    desired_speeds = np.array(
        [
            mission.waypoints.desired_speed(
                position,
                waypoint,
                max_speed,
                max_deceleration,
            )
            for position in history[:, :2]
        ]
    )

    # calculate speed error
    speed_errors = desired_speeds - history[:, 2]

    speed_rmse = np.sqrt(
        np.mean(speed_errors**2)
    )

    # calculate observed acceleration
    accelerations = np.diff(history[:, 2]) / dt

    min_acceleration = np.min(accelerations)
    max_acceleration = np.max(accelerations)

    # calculate simulation time
    final_time = (len(history) - 1) * dt

    # calculate actuator intervention
    control_difference = (
        result.requested_controls
        - result.applied_controls
    )

    acceleration_intervention = np.abs(
        control_difference[:, 0]
    )

    turn_rate_intervention = np.abs(
        control_difference[:, 1]
    )

    max_acceleration_intervention = np.max(
        acceleration_intervention
    )

    mean_acceleration_intervention = np.mean(
        acceleration_intervention
    )

    max_turn_rate_intervention = np.max(
        turn_rate_intervention
    )

    mean_turn_rate_intervention = np.mean(
        turn_rate_intervention
    )

    # calculate actuator saturation rates
    acceleration_saturated = (
        result.requested_controls[:, 0]
        != result.applied_controls[:, 0]
    )

    turn_rate_saturated = (
        result.requested_controls[:, 1]
        != result.applied_controls[:, 1]
    )

    acceleration_saturation_rate = np.mean(
        acceleration_saturated
    )

    turn_rate_saturation_rate = np.mean(
        turn_rate_saturated
    )

    # print experiment results
    print(f"Speed controller gain: {speed_kp:.2f}")
    print(f"Final position: {final_position}")
    print(f"Final speed: {final_speed:.3f} m/s")
    print(f"Final distance: {final_distance:.3f} m")
    print(f"Minimum distance: {minimum_distance:.3f} m")
    print(f"Final desired speed: {final_desired_speed:.3f} m/s")
    print(f"Speed RMSE: {speed_rmse:.3f} m/s")
    print(f"Minimum acceleration: {min_acceleration:.3f} m/s²")
    print(f"Maximum acceleration: {max_acceleration:.3f} m/s²")
    print(f"Final time: {final_time:.3f} s")
    print(
        f"Acceleration saturation rate: "
        f"{acceleration_saturation_rate * 100:.1f}%"
    )
    print(
        f"Turn-rate saturation rate: "
        f"{turn_rate_saturation_rate * 100:.1f}%"
    )
    print(
        f"Max acceleration intervention: "
        f"{max_acceleration_intervention:.3f} m/s²"
    )
    print(
        f"Mean acceleration intervention: "
        f"{mean_acceleration_intervention:.3f} m/s²"
    )
    print(
        f"Max turn-rate intervention: "
        f"{max_turn_rate_intervention:.3f} rad/s"
    )
    print(
        f"Mean turn-rate intervention: "
        f"{mean_turn_rate_intervention:.3f} rad/s"
    )

    return ExperimentResult(
        simulation=result,
        history=history,
        desired_speeds=desired_speeds,
        dt=dt,
        speed_kp=speed_kp,
        final_position=final_position,
        final_distance=final_distance,
        final_speed=final_speed,
        final_desired_speed=final_desired_speed,
        minimum_distance=minimum_distance,
        speed_rmse=speed_rmse,
        min_acceleration=min_acceleration,
        max_acceleration=max_acceleration,
        final_time=final_time,
        acceleration_saturation_rate=acceleration_saturation_rate,
        turn_rate_saturation_rate=turn_rate_saturation_rate,
        max_acceleration_intervention=max_acceleration_intervention,
        mean_acceleration_intervention=mean_acceleration_intervention,
        max_turn_rate_intervention=max_turn_rate_intervention,
        mean_turn_rate_intervention=mean_turn_rate_intervention,
    )


if __name__ == "__main__":
    speed_gains = [-2.0, -1.0, -0.5, 0.0, 0.25, 0.5, 1.0, 2.0]

    experiments = []

    for speed_kp in speed_gains:
        experiment = run_experiment(speed_kp)
        experiments.append(experiment)

    print("\nGain comparison")
    print("-" * 80)

    print(
        f"{'Kp':>6} "
        f"{'RMSE':>10} "
        f"{'Final Dist.':>12} "
        f"{'Accel Sat.':>12} "
        f"{'Mean Int.':>12} "
        f"{'Time':>10}"
    )

    print("-" * 80)

    for experiment in experiments:
        print(
            f"{experiment.speed_kp:>6.2f} "
            f"{experiment.speed_rmse:>10.3f} "
            f"{experiment.final_distance:>12.3f} "
            f"{experiment.acceleration_saturation_rate * 100:>11.1f}% "
            f"{experiment.mean_acceleration_intervention:>12.3f} "
            f"{experiment.final_time:>10.3f}"
        )

    history = experiment.history
    desired_speeds = experiment.desired_speeds
    dt = experiment.dt

    x = experiment.history[:, 0]
    y = experiment.history[:, 1]
    v = experiment.history[:, 2]
    time = np.arange(len(experiment.history)) * dt

    # plot desired vs actual speed
    plt.figure()

    plt.plot(
        time,
        v,
        label="Actual speed",
    )

    plt.plot(
        time,
        desired_speeds,
        "--",
        label="Desired speed",
    )

    plt.xlabel("Time (s)")
    plt.ylabel("Speed (m/s)")
    plt.title("Actual vs Desired Speed over Time")
    plt.legend()
    plt.grid()

    plt.show()

    # plot trajectory
    plt.figure()

    plt.plot(x, y)

    plt.xlabel("x position (m)")
    plt.ylabel("y position (m)")
    plt.title("Aircraft Trajectory")
    plt.grid()

    plt.show()