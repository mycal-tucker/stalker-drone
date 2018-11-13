from cinematic_waypoints.waypoint_generator.waypoint_generator_abc import WaypointGenerator
from utils.bounding_box import BoundingBox
from utils.drone_state import DroneState
import numpy as np
import math


# WaypointGenerator object that tries to track a single fixed bounding box, without implementing
# more complex logic like human state, or fixed flight patterns.
class FixedBBWaypointGenerator(WaypointGenerator):
    # FIXME: this target_bb is almost certainly wrong. We need to sync up with the human-tracker team to learn
    # what sorts of values we can expect.
    def __init__(self, target_bb=BoundingBox((10, 10), (20, 20))):
        self.target_bb = target_bb
        # This number should be really small, since we're converting area in pixels to a displacement in meters.
        self.area_gain = 0.0005
        self.z_gain = 0.5
        self.yaw_gain = 0.5

    # Expose target_bb for testability.
    def get_target_bb(self):
        return self.target_bb

    # Ignores the drone_state argument. The only logic here is to try to make the bounding_box match
    # self.target_bounding_box.
    # The rough idea is:
    # if the box is too small, fly forward (too big -> fly backward)
    # if the box is too low, fly down (too high -> fly up)
    # if the box is left of center, turn left (right of center, turn right)
    # if there is no box, return the current drone state to hover.
    def generate_waypoints(self, bounding_box, drone_state):
        if bounding_box is None:
            return [drone_state]
        centroid_diff = np.asarray(bounding_box.get_centroid()) - np.asarray(self.target_bb.get_centroid())
        area_diff = bounding_box.get_area() - self.target_bb.get_area()

        yaw_control = -1 * self.yaw_gain * centroid_diff[0]
        z_control = self.z_gain * centroid_diff[1]

        forward_control = self.area_gain * area_diff
        current_roll, current_pitch, current_yaw = drone_state.get_attitude()
        # FIXME: Note, I'm not yet sure how we'll calculate angles, so it's very possible that I've
        # got the sin, cosine, or the signs messed up. Hopefully, this will be easy to detect, at least.
        # Right now I'm operating as if 0 yaw means facing along the x axis, and increasing yaw means turning
        # towards the y axis.
        area_x_delta = -1 * forward_control * math.cos(current_yaw)
        area_y_delta = -1 * forward_control * math.sin(current_yaw)

        current_x, current_y, current_z = drone_state.get_position()
        target_position = (current_x + area_x_delta, current_y + area_y_delta, current_z + z_control)
        target_attitude = (0, 0, current_yaw + yaw_control)  # Always aim for 0 pitch and roll.

        # Recall that this method must return a list of DroneStates, so even if we only generate one, we must
        # put it inside a list.
        return [DroneState(target_position[0], target_position[1], target_position[2],
                           target_attitude[0], target_attitude[1], target_attitude[2],
                           0, 0, 0, 0, 0, 0)]  # All target velocities are 0
