import numpy as np
import matplotlib.pyplot as plt
from robust_flight.control.speed import SpeedController
from robust_flight.dynamics.aircraft import Aircraft
from robust_flight.simulation.engine import simulate
from robust_flight.mission.mission import Mission
from robust_flight.control.heading import HeadingController
from robust_flight.control.speed import SpeedController

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
    
    desired_speed = mission.waypoints.desired_speed(state[:2], waypoint, max_speed=20.0, max_deceleration=2.0)
    
    speed_error = desired_speed - state[2]
    
    acceleration = speed_controller.control(speed_error)
    
    heading_control = heading_controller.control(heading_error)

    return np.array([acceleration, heading_control])

def termination_condition(state):
    mission.update(state[:2])
    return mission.is_mission_complete()

# desired_heading = waypoints.desired_heading(state[:2], waypoint)

# heading_error = waypoints.heading_error(state[3], desired_heading)

# control = heading_controller.control(heading_error)

history = simulate(aircraft, control_function, 10.0, 0.1, termination_condition)

x = history[:, 0]
y = history[:, 1]

plt.plot(x, y)
plt.xlabel('x position (m)')
plt.ylabel('y position (m)')
plt.title('Aircraft Trajectory')
plt.show()