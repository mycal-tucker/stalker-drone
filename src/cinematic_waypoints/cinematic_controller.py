from cinematic_waypoints.waypoint_generator.fixed_bb_waypoint_generator import FixedBBWaypointGenerator
from utils.bounding_box import BoundingBox
from utils.person_state import PersonState
import numpy as np
import numpy.linalg
import math


# Class for generating cinematic waypoint.
# Consumes updates on bounding boxes for humans and updates of drone state.
# Produces a series of drone states in the global reference frame for the
# drone to fly to. It is up to other components to take care of the actual
# control to fly the drone to the waypoints smoothly.
class CinematicController:
    # There are a couple tunable parameters, which are accepted as arguments with default
    # values. Often, the parameters are only used in one method, so they don't need to
    # be class variables, but this style allows us to keep all tunable parameters in the same
    # place, and it improves testability (e.g. tests can set bb_filter_gamma to whatever value
    # they want).
    def __init__(self, waypoint_generator=FixedBBWaypointGenerator(), bb_filter_gamma=0.9,
                 bb_match_dimension_importance=0.5, margin=0.25, person_predictor=None):
        self.latest_bounding_box = None
        self.smoothed_bounding_box = None
        self.bb_filter_gamma = bb_filter_gamma
        self.bb_match_dimension_importance = bb_match_dimension_importance
        self.margin = margin # margin is how close we want the drone to be to the desired state

        self.latest_drone_state = None
        self.cinematic_waypoints = None
        # Delegate the task of generating waypoints to the waypoint_generator, which must implement
        # the WaypointGenerator interface.
        self.waypoint_generator = waypoint_generator
        self.person_predictor = person_predictor

    def set_waypoint_generator(self, waypoint_generator):
        print("Why would you update the waypoint generator? Only call this from a test.")
        self.waypoint_generator = waypoint_generator

    # Accepts a list of bounding boxes, selects the single bounding box it thinks is the best
    # match to the bounding box it was already tracking, and then updates the state variables
    # to reflect the new bounding box info.
    # Future iterations could keep the information about all the bounding boxes to allow for
    # tracking multiple people, or guaranteeing that we don't run into other people as we track
    # just one person.
    # If the input is degenerate (None or empty), updates the latest and smooth bounding box to None,
    # and rely on the waypoint generator to do something intelligent as a result.
    def update_latest_bbs(self, all_bounding_boxes):
        if all_bounding_boxes is None or len(all_bounding_boxes) == 0:
            print("Error, no bounding boxes provided. This case is not handled.")
            print("For now, assume that the bounding box just disappeared, so don't change the tracked"
                  " bounding box at all. Other valid approaches would be to set the latest value to"
                  " null but not update teh smoothed bounding box.")
            self.latest_bounding_box = None
            self.smoothed_bounding_box = None
            return
        best_match_bb = self.identify_best_bb_match(all_bounding_boxes)
        self.latest_bounding_box = best_match_bb
        # Set the smoothed bounding box, looking out for initialization
        # conditions.
        if self.smoothed_bounding_box is None:
            self.smoothed_bounding_box = best_match_bb
        else:
            self.smoothed_bounding_box = self.compute_low_pass_filter_bb()
        # Compute a person state from the bounding box
        if self.person_predictor is not None:
            person_state = PersonState.get_person_state_from_bb(self.latest_drone_state, self.smoothed_bounding_box)
            self.person_predictor.add_person_state(person_state)

    # Given a list of bounding boxes, returns the one that best matches the currently-tracked
    # bounding box.
    # If no bounding box is currently tracked, returns the largest bounding box in the list.
    def identify_best_bb_match(self, bounding_boxes):
        # If not already tracking a bounding box, return the largest box
        if self.smoothed_bounding_box is None:
            return CinematicController.identify_largest_bb(bounding_boxes)
        centroid_importance = 1 - self.bb_match_dimension_importance

        best_bb = None
        min_error = math.inf
        for bb in bounding_boxes:
            # Use Euclidean distance of dimensions and centroid to compute the "error"
            bb_dimensions = np.asarray(bb.get_dimensions())
            dimension_error = numpy.linalg.norm(bb_dimensions - np.asarray(self.smoothed_bounding_box.get_dimensions()))
            bb_centroid = np.asarray(bb.get_centroid())
            centroid_error = numpy.linalg.norm(bb_centroid - np.asarray(self.smoothed_bounding_box.get_centroid()))

            error = self.bb_match_dimension_importance * dimension_error + centroid_importance * centroid_error
            if error < min_error:
                min_error = error
                best_bb = bb

        return best_bb

    # Returns the bounding box from the list provided with the largest area.
    @staticmethod
    def identify_largest_bb(bounding_boxes):
        largest_bb = None
        largest_area = -1
        for bb in bounding_boxes:
            bb_area = bb.get_area()
            if bb_area > largest_area:
                largest_area = bb_area
                largest_bb = bb
        return largest_bb

    def update_latest_drone_state(self, drone_state):
        self.latest_drone_state = drone_state

    def update_cinematic_waypoints(self, cinematic_waypoints):
        self.cinematic_waypoints = cinematic_waypoints

    # Uses a geometric low-pass filter to smooth out human bounding boxes
    def compute_low_pass_filter_bb(self):
        latest_width, latest_height = self.latest_bounding_box.get_dimensions()
        latest_x, latest_y = self.latest_bounding_box.get_centroid()

        smoothed_width, smoothed_height = self.smoothed_bounding_box.get_dimensions()
        smoothed_x, smoothed_y = self.smoothed_bounding_box.get_centroid()

        new_width = smoothed_width * self.bb_filter_gamma + latest_width * (1 - self.bb_filter_gamma)
        new_height = smoothed_height * self.bb_filter_gamma + latest_height * (1 - self.bb_filter_gamma)
        new_x = smoothed_x * self.bb_filter_gamma + latest_x * (1 - self.bb_filter_gamma)
        new_y = smoothed_y * self.bb_filter_gamma + latest_y * (1 - self.bb_filter_gamma)
        return BoundingBox((new_width, new_height), (new_x, new_y))

    def generate_waypoints(self):
        if self.smoothed_bounding_box is None or self.latest_drone_state is None:
            print("Warning: asked to generate waypoints, but some of the current states are None.")
        if self.cinematic_waypoints is None:
            self.cinematic_waypoints = self.waypoint_generator.generate_waypoints(self.smoothed_bounding_box,
                                                                                  self.latest_drone_state,
                                                                                  self.person_predictor)
            return self.cinematic_waypoints
        # If we reach this point, we already have waypoints cached, so we need to check against them.
        waypoint = self.cinematic_waypoints[0]
        x_offset = self.latest_drone_state.x - waypoint.x
        y_offset = self.latest_drone_state.y - waypoint.y
        z_offset = self.latest_drone_state.z - waypoint.z
        # Check if close to the next waypoint. If so, delete the waypoint and update the rest
        # of the trajectory. If not, just return the current waypoints.
        if x_offset < self.margin and y_offset < self.margin and z_offset < self.margin:
            del self.cinematic_waypoints[0]
            if len(self.cinematic_waypoints) == 0:
                self.cinematic_waypoints = self.waypoint_generator.generate_waypoints(self.smoothed_bounding_box,
                                                                                      self.latest_drone_state,
                                                                                      self.person_predictor)
        return self.cinematic_waypoints

