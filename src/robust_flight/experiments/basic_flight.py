import matplotlib.pyplot as plt
import numpy as np

from robust_flight.control.heading import HeadingController
from robust_flight.control.speed import SpeedController
from robust_flight.dynamics.aircraft import Aircraft
from robust_flight.mission.mission import Mission
from robust_flight.simulation.engine import simulate

# simulation parameters
duration = 10.0
dt = 0.1

# guidance parameters
max_speed = 20.0
max_deceleration = 2.0

# controller gains
heading_kp = 2.0
speed_kp = 2.0

# initial state
state = np.array([0.0, 0.0, 10.0, 0.0])

# create an aircraft instance from the class
aircraft = Aircraft(state)

# define the waypoint and create a Waypoints instance
waypoint = np.array([50.0, 30.0])
mission = Mission([waypoint])

# create the heading and speed controllers
heading_controller = HeadingController(kp=heading_kp)
speed_controller = SpeedController(kp=speed_kp)


def control_function(state):
    waypoint = mission.current_waypoint()

    if waypoint is None:
        return np.array([0.0, 0.0])

    desired_heading = mission.waypoints.desired_heading(state[:2], waypoint)

    heading_error = mission.waypoints.heading_error(state[3], desired_heading)

    desired_speed = mission.waypoints.desired_speed(
        state[:2], waypoint, max_speed, max_deceleration
    )

    speed_error = desired_speed - state[2]

    acceleration = speed_controller.control(speed_error)

    heading_control = heading_controller.control(heading_error)

    return np.array([acceleration, heading_control])


def termination_condition(state):
    mission.update(state[:2])
    return mission.is_mission_complete()


history = simulate(aircraft, control_function, duration, dt, termination_condition)

# check metrics
final_position = history[-1, :2]
final_distance = mission.waypoints.distance_to_waypoint(final_position, waypoint)
final_speed = history[-1, 2]
final_desired_speed = mission.waypoints.desired_speed(
    final_position,
    waypoint,
    max_speed,
    max_deceleration,
)
distances = np.linalg.norm(history[:, :2] - waypoint, axis=1)
desired_speeds = np.array([
    mission.waypoints.desired_speed(
        position,
        waypoint,
        max_speed,
        max_deceleration,
    )
    for position in history[:, :2]
])

speed_errors = desired_speeds - history[:, 2]
speed_rmse = np.sqrt(np.mean(speed_errors**2))
minimum_distance = np.min(distances)
accelerations = np.diff(history[:, 2]) / dt
min_acceleration = np.min(accelerations)
max_acceleration = np.max(accelerations)
final_time = (len(history) - 1) * dt

print(
    f"Final position: {final_position}, \n"
    f"speed: {final_speed:.3f} m/s, \n"
    f"Minimum distance to waypoint: {minimum_distance:.3f} m, \n"
    f"distance to waypoint: {final_distance:.3f} m, \n"
    f"desired speed: {final_desired_speed:.3f} m/s, \n"
    f"max deceleration: {min_acceleration:.3f} m/s², \n"
    f"max acceleration: {max_acceleration:.3f} m/s², \n"
    f"final time: {final_time:.3f} s, \n"
    f"speed tracking RMSE: {speed_rmse:.3f} m/s, \n"
)

x = history[:, 0]
y = history[:, 1]
v = history[:, 2]
time = np.arange(len(history)) * dt

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
