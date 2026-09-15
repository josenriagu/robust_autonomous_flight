import numpy as np

class Waypoints:
    def __init__(self, points):
        self.points = np.array(points)

    # def closest_waypoint(self, position):
    #     position = np.array(position)
    #     distances = np.linalg.norm(self.points - position, axis=1)
    #     return np.argmin(distances)
    
    def desired_heading(self, position, waypoint):
        position = np.array(position)
        waypoint = np.array(waypoint)
        direction = waypoint - position
        return np.arctan2(direction[1], direction[0])
    
    def heading_error(self, current_heading, desired_heading):
        error = desired_heading - current_heading
        return (error + np.pi) % (2 * np.pi) - np.pi