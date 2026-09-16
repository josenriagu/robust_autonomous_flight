import matplotlib.pyplot as plt
import numpy as np

from robust_flight.control.heading import HeadingController
from robust_flight.control.speed import SpeedController
from robust_flight.dynamics.aircraft import Aircraft
from robust_flight.mission.mission import Mission
from robust_flight.simulation.engine import simulate

# initial state
state = np.array([0.0, 0.0, 10.0, 0.0])

# create an aircraft instance from the class
aircraft = Aircraft(state)

# define the waypoint and create a Waypoints instance
waypoint = np.array([50.0, 30.0])
mission = Mission([waypoint])

# create the heading and speed controllers
heading_controller = HeadingController(kp=2.0)
speed_controller = SpeedController(kp=0.5)


def control_function(state):
    waypoint = mission.current_waypoint()

    if waypoint is None:
        return np.array([0.0, 0.0])

    desired_heading = mission.waypoints.desired_heading(state[:2], waypoint)

    heading_error = mission.waypoints.heading_error(state[3], desired_heading)

    desired_speed = mission.waypoints.desired_speed(
        state[:2], waypoint, max_speed=20.0, max_deceleration=2.0
    )

    speed_error = desired_speed - state[2]

    acceleration = speed_controller.control(speed_error)

    heading_control = heading_controller.control(heading_error)

    return np.array([acceleration, heading_control])


def termination_condition(state):
    mission.update(state[:2])
    return mission.is_mission_complete()


history = simulate(aircraft, control_function, 10.0, 0.1, termination_condition)

# check metrics
final_position = history[-1, :2]
final_distance = mission.waypoints.distance_to_waypoint(final_position, waypoint)
final_speed = history[-1, 2]
final_desired_speed = mission.waypoints.desired_speed(
    final_position,
    waypoint,
    max_speed=20.0,
    max_deceleration=2.0,
)
distances = np.linalg.norm(history[:, :2] - waypoint, axis=1)
desired_speeds = np.array([
    mission.waypoints.desired_speed(
        position,
        waypoint,
        max_speed=20.0,
        max_deceleration=2.0,
    )
    for position in history[:, :2]
])
minimum_distance = np.min(distances)
accelerations = np.diff(history[:, 2]) / 0.1
min_acceleration = np.min(accelerations)
max_acceleration = np.max(accelerations)
final_time = (len(history) - 1) * 0.1

print(
    f"Final position: {final_position}, speed: {final_speed:.3f} m/s, "
    f"Minimum distance to waypoint: {minimum_distance:.3f} m, "
    f"distance to waypoint: {final_distance:.3f} m, "
    f"desired speed: {final_desired_speed:.3f} m/s, "
    f"max deceleration: {min_acceleration:.3f} m/s², "
    f"max acceleration: {max_acceleration:.3f} m/s², "
    f"final time: {final_time:.3f} s"
)

x = history[:, 0]
y = history[:, 1]
v = history[:, 2]
time = np.arange(len(history)) * 0.1

# plot desired vs actual speed
plt.plot(time, v, label="Actual speed")
plt.plot(time, desired_speeds, "--", label="Desired speed")
plt.xlabel("Time (s)")
plt.ylabel("Speed (m/s)")
plt.title("Actual vs Desired Speed over Time")
plt.legend()
plt.show()

# plot trajectory
plt.plot(x, y)
plt.xlabel("x position (m)")
plt.ylabel("y position (m)")
plt.title("Aircraft Trajectory")
plt.show()
