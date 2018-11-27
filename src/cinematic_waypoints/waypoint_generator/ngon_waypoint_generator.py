from cinematic_waypoints.waypoint_generator.waypoint_generator_abc import WaypointGenerator
from utils.drone_state import DroneState
from utils.person_state import PersonState
import math

# WaypointGenerator object that makes the drone fly in an NGon facing the middle, completely
# ignoring the bounding-box information it gets.
class NGonWaypointGenerator(WaypointGenerator):
    # Accept as arguments the number of waypoints and the desired "radius" of the n-gon, which
    # is the distance from all vertices to the center of the waypoint.
    def __init__(self, n=8, radius=1):
        self.n = n
        self.radius = radius

    # Ignores the bounding_box argument.
    # The only logic here is to try to make the drone fly in an n-gon by generating waypoints
    # at the vertices, starting from the current state.
    # This could be extended to estimate the distance to the person (from the bounding box) and
    # then try to make the drone fly in an n-gon with the person at the center.
    def generate_waypoints(self, bounding_box, drone_state, person_predictor=None):
        current_x, current_y, current_z = drone_state.get_position()
        current_roll, current_pitch, current_yaw = drone_state.get_attitude()
        # Compute the desired center of the n-gon.
        # Note how we could easily adapt this code to use the person as the center if we can
        # somehow get a person's position from the bounding box.
        center_x = current_x + math.cos(current_yaw) * self.radius
        center_y = current_y + math.sin(current_yaw) * self.radius

        time_to_project = 10
        timesteps = [i * time_to_project / self.n for i in range(self.n + 1)]
        current_person_state = PersonState(center_x, center_y)
        predicted_person_states = [current_person_state for _ in range(self.n + 1)]
        if person_predictor is not None:
            predicted_person_states = person_predictor.predict_next_person_state(timesteps)

        target_waypoints = []
        angle_per_segment = 2.0 * math.pi / self.n
        for i in range(1, self.n + 1):  # Don't include current state as a starting waypoint
            predicted_person_state = predicted_person_states[i]
            # new_x = math.cos(angle_per_segment * i + math.pi + current_yaw) * self.radius + center_x
            # new_y = math.sin(angle_per_segment * i + math.pi + current_yaw) * self.radius + center_y
            new_x = math.cos(angle_per_segment * i + math.pi + current_yaw) * self.radius + predicted_person_state.x
            new_y = math.sin(angle_per_segment * i + math.pi + current_yaw) * self.radius + predicted_person_state.y
            new_theta = current_yaw + i * angle_per_segment  # Check if need to crop to be within [-2pi, 2pi]

            target_waypoints.append(DroneState(new_x, new_y, current_z,
                                               current_roll, current_pitch, new_theta,
                                               0, 0, 0, 0, 0, 0))  # All velocities are 0
        # target_waypoints.append(drone_state)  # end up back where you started.
        return target_waypoints
