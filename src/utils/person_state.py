import math


# Representation of a person state, which for now is just x and y location.
# The key method is estimating the state from a drone state and a bounding box.
# There's lots of room for calibration and more sophisticated calculation.
class PersonState:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.radius = 0.5

    # Estimate an x, y position from a bounding_box and a drone state. We need the
    # drone state as a sort of reference frame, and then we project into a new point
    # in space using the bounding_box.
    @staticmethod
    def get_person_state_from_bb(drone_state, bounding_box):
        # For first implementation, just estimate distance based on size. This is a bit dumb.
        bb_area = bounding_box.get_area()
        distance_to_bb = 100.0 / (bb_area * bb_area)
        # FIXME: this angle calculation also needs calibration.
        angle_to_center_bb = math.atan2(bounding_box.get_centroid()[0], 100)

        # Now project forward.
        x_delta = math.cos(drone_state.yaw + angle_to_center_bb) * distance_to_bb
        y_delta = math.sin(drone_state.yaw + angle_to_center_bb) * distance_to_bb

        return PersonState(drone_state.x + x_delta, drone_state.y + y_delta)

    # Return the state of the person 
    def get_person_state(self):
        return self.x, self.y, self.radius

    def __eq__(self, other):
        if not isinstance(other, PersonState):
            return False
        return other.x == self.x and other.y == self.y and other.radius == self.radius

    def __hash__(self):
        return 17 * self.x.hash() + self.y.hash() + 57 * self.radius.hash()
