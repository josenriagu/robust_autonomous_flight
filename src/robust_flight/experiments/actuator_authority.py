from dataclasses import dataclass

import numpy as np

from robust_flight.control.heading import HeadingController
from robust_flight.control.speed import SpeedController
from robust_flight.dynamics.actuator import Actuator
from robust_flight.dynamics.aircraft import Aircraft
from robust_flight.experiments.basic_flight import ExperimentResult
from robust_flight.mission.mission import Mission
from robust_flight.simulation.engine import simulate


@dataclass
class ActuatorAuthorityResult(ExperimentResult):
    acceleration_effort: float
    turn_rate_effort: float


def run_case(min_acceleration, max_acceleration, max_turn_rate):
    duration = 10.0
    dt = 0.1
    max_speed = 20.0
    max_deceleration = 2.0
    heading_kp = 2.0
    speed_kp = 1.0

    aircraft = Aircraft(np.array([0.0, 0.0, 10.0, 0.0]))
    waypoint = np.array([50.0, 30.0])
    mission = Mission([waypoint])
    heading_controller = HeadingController(kp=heading_kp)
    speed_controller = SpeedController(kp=speed_kp)
    actuator = Actuator(
        min_acceleration=min_acceleration,
        max_acceleration=max_acceleration,
        max_turn_rate=max_turn_rate,
    )

    def control_function(state):
        current_waypoint = mission.current_waypoint()

        if current_waypoint is None:
            return np.array([0.0, 0.0])

        desired_heading = mission.waypoints.desired_heading(
            state[:2],
            current_waypoint,
        )
        heading_error = mission.waypoints.heading_error(
            state[3],
            desired_heading,
        )
        desired_speed = mission.waypoints.desired_speed(
            state[:2],
            current_waypoint,
            max_speed,
            max_deceleration,
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
        duration,
        dt,
        termination_condition,
        actuator=actuator,
    )
    history = simulation.states

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
    speed_errors = desired_speeds - history[:, 2]
    distances = np.linalg.norm(history[:, :2] - waypoint, axis=1)
    final_position = history[-1, :2]
    final_distance = mission.waypoints.distance_to_waypoint(final_position, waypoint)

    control_difference = simulation.requested_controls - simulation.applied_controls
    acceleration_intervention = np.abs(control_difference[:, 0])
    turn_rate_intervention = np.abs(control_difference[:, 1])
    acceleration_saturated = control_difference[:, 0] != 0.0
    turn_rate_saturated = control_difference[:, 1] != 0.0

    applied_controls = simulation.applied_controls
    return ActuatorAuthorityResult(
        simulation=simulation,
        history=history,
        desired_speeds=desired_speeds,
        speed_errors=speed_errors,
        max_speed_error=float(np.max(np.abs(speed_errors))),
        dt=dt,
        speed_kp=speed_kp,
        final_position=final_position,
        final_distance=float(final_distance),
        final_speed=float(history[-1, 2]),
        final_desired_speed=float(desired_speeds[-1]),
        minimum_distance=float(np.min(distances)),
        speed_rmse=float(np.sqrt(np.mean(speed_errors**2))),
        min_acceleration=float(np.min(np.diff(history[:, 2]) / dt)),
        max_acceleration=float(np.max(np.diff(history[:, 2]) / dt)),
        final_time=float((len(history) - 1) * dt),
        acceleration_saturation_rate=float(np.mean(acceleration_saturated)),
        turn_rate_saturation_rate=float(np.mean(turn_rate_saturated)),
        max_acceleration_intervention=float(np.max(acceleration_intervention)),
        mean_acceleration_intervention=float(np.mean(acceleration_intervention)),
        max_turn_rate_intervention=float(np.max(turn_rate_intervention)),
        mean_turn_rate_intervention=float(np.mean(turn_rate_intervention)),
        acceleration_effort=float(np.sum(applied_controls[:, 0] ** 2) * dt),
        turn_rate_effort=float(np.sum(applied_controls[:, 1] ** 2) * dt),
    )


def print_sweep(title, cases, authority_label):
    print(f"\n{title}")
    print(
        f"{authority_label:>18} {'Speed RMSE':>11} {'Max speed err.':>15} "
        f"{'Final dist.':>12} {'Min dist.':>11} {'Final speed':>12} "
        f"{'Time':>8} {'Accel sat.':>11} {'Turn sat.':>10} "
        f"{'Mean accel int.':>16} {'Max accel int.':>15} "
        f"{'Mean turn int.':>15} {'Max turn int.':>14} "
        f"{'Accel effort':>13} {'Turn effort':>12}"
    )

    for authority, result in cases:
        print(
            f"{authority:>18} "
            f"{result.speed_rmse:>11.3f} "
            f"{result.max_speed_error:>15.3f} "
            f"{result.final_distance:>12.3f} "
            f"{result.minimum_distance:>11.3f} "
            f"{result.final_speed:>12.3f} "
            f"{result.final_time:>8.2f} "
            f"{result.acceleration_saturation_rate * 100:>10.1f}% "
            f"{result.turn_rate_saturation_rate * 100:>9.1f}% "
            f"{result.mean_acceleration_intervention:>16.3f} "
            f"{result.max_acceleration_intervention:>15.3f} "
            f"{result.mean_turn_rate_intervention:>15.3f} "
            f"{result.max_turn_rate_intervention:>14.3f} "
            f"{result.acceleration_effort:>13.3f} "
            f"{result.turn_rate_effort:>12.3f}"
        )


def main():
    acceleration_limits = [
        (-1.0, 1.5),
        (-1.5, 2.0),
        (-2.0, 3.0),
        (-3.0, 4.0),
        (-4.0, 5.0),
    ]
    acceleration_cases = [
        (
            f"({minimum:.1f}, {maximum:.1f})",
            run_case(minimum, maximum, np.pi / 3),
        )
        for minimum, maximum in acceleration_limits
    ]

    max_turn_rates = [
        ("π/12", np.pi / 12),
        ("π/8", np.pi / 8),
        ("π/6", np.pi / 6),
        ("π/4", np.pi / 4),
        ("π/3", np.pi / 3),
        ("π/2", np.pi / 2),
    ]
    turn_rate_cases = [
        (
            f"{label} rad/s",
            run_case(-2.0, 3.0, turn_rate),
        )
        for label, turn_rate in max_turn_rates
    ]

    print_sweep("Acceleration authority sweep", acceleration_cases, "Accel limits")
    print_sweep("Turn-rate authority sweep", turn_rate_cases, "Max turn rate")


if __name__ == "__main__":
    main()
