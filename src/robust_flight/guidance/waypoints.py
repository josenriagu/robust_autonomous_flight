import numpy as np


class Waypoints:
    def __init__(self, points):
        self.points = np.array(points)

    def desired_heading(self, position, waypoint):
        position = np.array(position)
        waypoint = np.array(waypoint)
        direction = waypoint - position
        return np.arctan2(direction[1], direction[0])

    def heading_error(self, current_heading, desired_heading):
        error = desired_heading - current_heading
        return (error + np.pi) % (2 * np.pi) - np.pi

    def distance_to_waypoint(self, position, waypoint):
        position = np.array(position)
        waypoint = np.array(waypoint)
        return np.linalg.norm(waypoint - position)

    def desired_speed(self, position, waypoint, max_speed, max_deceleration):
        distance = self.distance_to_waypoint(position, waypoint)
        braking_speed = np.sqrt(2 * max_deceleration * distance)
        return min(max_speed, braking_speed) if distance > 0 else 0.0
