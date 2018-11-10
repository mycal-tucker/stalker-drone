import abc


# Abstract Base Class (abc) for enforcing that all waypoint generators implement the same interface.
# For now, that's just one method, generate_waypoints(), which returns a list of waypoints (DroneStates)
# for a drone to follow.
class WaypointGenerator(abc.ABC):
    @abc.abstractmethod
    def generate_waypoints(self, bounding_box, drone_state):
        pass
