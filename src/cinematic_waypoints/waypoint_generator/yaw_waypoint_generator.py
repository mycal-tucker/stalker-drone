from cinematic_waypoints.waypoint_generator.waypoint_generator_abc import WaypointGenerator
from utils.drone_state import DroneState


# WaypointGenerator object that only allows the drone to yaw in place.
class YawWaypointGenerator(WaypointGenerator):
    def __init__(self, target_centroid_x=320):  # Images are 640 pixels across
        # This should correspond to the x value of a pixel right in the middle of a photo.
        self.target_centroid_x = target_centroid_x
        # Parameter for gain for how quickly the drone should yaw.
        self.yaw_gain = 0.5

    # Exposed for testing.
    def get_target_centroid_x(self):
        return self.target_centroid_x

    # Ignores the drone_state argument. The only logic here is to try to yaw in place
    # to get the x component of the centroid in the middle of the picture.
    # The rough idea is:
    # if the box is left of center, turn left (right of center, turn right)
    # if there is no box, return the current drone state to hover.
    def generate_waypoints(self, bounding_box, drone_state, person_predictor=None):
        if bounding_box is None:
            return [drone_state]
        x_diff = bounding_box.get_centroid()[0] - self.target_centroid_x

        yaw_control = -1 * self.yaw_gain * x_diff

        current_roll, current_pitch, current_yaw = drone_state.get_attitude()

        target_position = drone_state.get_position()
        target_attitude = (0, 0, current_yaw + yaw_control)  # Always aim for 0 pitch and roll.

        # Recall that this method must return a list of DroneStates, so even if we only generate one, we must
        # put it inside a list.
        return [DroneState(target_position[0], target_position[1], target_position[2],
                           target_attitude[0], target_attitude[1], target_attitude[2],
                           0, 0, 0, 0, 0, 0)]  # All target velocities are 0
