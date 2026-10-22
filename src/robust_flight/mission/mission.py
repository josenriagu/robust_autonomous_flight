from robust_flight.guidance.waypoints import Waypoints


class Mission:
    def __init__(self, waypoints, arrival_radius=1.0):
        self.waypoints = Waypoints(waypoints)
        self.arrival_radius = arrival_radius
        self.current_waypoint_index = 0

    def current_waypoint(self):
        if (
            self.current_waypoint_index is not None
            and self.current_waypoint_index < len(self.waypoints.points)
        ):
            return self.waypoints.points[self.current_waypoint_index]
        return None

    def has_reached_waypoint(self, position, waypoint):
        distance = self.waypoints.distance_to_waypoint(position, waypoint)
        return distance <= self.arrival_radius

    def advance_to_next_waypoint(self):
        if self.current_waypoint_index < len(self.waypoints.points) - 1:
            self.current_waypoint_index += 1
        else:
            self.current_waypoint_index = None

    def is_mission_complete(self):
        return self.current_waypoint_index is None

    def update(self, position):
        if not self.is_mission_complete():
            waypoint = self.current_waypoint()
            if self.has_reached_waypoint(position, waypoint):
                self.advance_to_next_waypoint()
                return True
        return False
