import numpy as np
import matplotlib.pyplot as plt
from robust_flight.dynamics.aircraft import Aircraft
from robust_flight.simulation.engine import simulate
from robust_flight.guidance.waypoints import Waypoints
from robust_flight.control.heading import HeadingController

state = np.array([0.0, 0.0, 10.0, 0.0])

aircraft = Aircraft(state)


waypoint = np.array([50.0, 30.0])
waypoints = Waypoints([waypoint])

heading_controller = HeadingController(kp=2.0)

def control_function(state):
    desired_heading = waypoints.desired_heading(state[:2], waypoint)

    heading_error = waypoints.heading_error(state[3], desired_heading)

    return np.array([2.0, heading_controller.control(heading_error)])

# desired_heading = waypoints.desired_heading(state[:2], waypoint)

# heading_error = waypoints.heading_error(state[3], desired_heading)

# control = heading_controller.control(heading_error)

history = simulate(aircraft, control_function, 10.0, 0.1)


x = history[:, 0]
y = history[:, 1]

plt.plot(x, y)
plt.xlabel('x position (m)')
plt.ylabel('y position (m)')
plt.title('Aircraft Trajectory')
plt.show()