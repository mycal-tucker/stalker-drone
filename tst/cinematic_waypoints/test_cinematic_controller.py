import unittest
from cinematic_waypoints.cinematic_controller import CinematicController
from cinematic_waypoints.waypoint_visualization import WaypointVisualization
from cinematic_waypoints.waypoint_generator.fixed_bb_waypoint_generator import FixedBBWaypointGenerator
from cinematic_waypoints.waypoint_generator.yaw_waypoint_generator import YawWaypointGenerator
from cinematic_waypoints.waypoint_generator.ngon_waypoint_generator import NGonWaypointGenerator
from utils.bounding_box import BoundingBox
from utils.drone_state import DroneState
from utils.environment import Environment
from utils.person_state import PersonState
from person_detection.person_predictor.polynomial_predictor import PolynomialPredictor
import math
import os
import time

# Define a few helpful variables common across a few tests
bb_10_10_0_0 = BoundingBox((10, 10), (0, 0))
bb_10_20_0_0 = BoundingBox((10, 20), (0, 0))
bb_20_10_0_0 = BoundingBox((20, 10), (0, 0))
bb_20_20_0_0 = BoundingBox((20, 20), (0, 0))
# Will often use this state as a starting point. Drone is at rest at the origin, facing along the +x direction.
# Tests may have to be updated if the understanding of yaw changes.
origin_drone_state = DroneState(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)


# Class for testing CinematicController class.
# Remember to start each test method with the name "test_..."
class TestCinematicController(unittest.TestCase):
    # Before each test, create a brand new cinematic controller to play with.
    def setUp(self):
        self.waypoint_generator = FixedBBWaypointGenerator()
        self.gamma = 0.9
        self.cinematic_controller = CinematicController(waypoint_generator=self.waypoint_generator,
                                                        bb_filter_gamma=self.gamma)

    # Example of a dummy test that you can use for creating lots of other tests.
    # Demonstrates how you can access information about the cinematic controller
    # in all the test methods.
    # def test_dummy(self):
    #     print(self.cinematic_controller.bb_filter_gamma)

    # Test that the smoothed bounding box gets updated correctly along the width dimension.
    def test_bb_low_pass_filter_dimensions1(self):
        self.cinematic_controller.update_latest_bbs([bb_10_10_0_0])
        # Check that smoothed and latest have been updated
        latest_bb = self.cinematic_controller.latest_bounding_box
        smoothed_bb = self.cinematic_controller.smoothed_bounding_box
        assert latest_bb == bb_10_10_0_0
        assert smoothed_bb == bb_10_10_0_0

        # Now add another box and check that width dimension updates correctly
        self.cinematic_controller.update_latest_bbs([bb_20_10_0_0])
        latest_bb = self.cinematic_controller.latest_bounding_box
        smoothed_bb = self.cinematic_controller.smoothed_bounding_box
        assert latest_bb == bb_20_10_0_0
        # Some trickier math for the smoothed bb
        assert smoothed_bb.get_dimensions()[0] == 11  # Update if gamma changes.
        assert smoothed_bb.get_dimensions()[1] == 10
        assert smoothed_bb.get_centroid() == bb_20_10_0_0.get_centroid()

    # TODO, one could clearly add lots more tests for varying the dimensions in other directions, for varying
    # the centroid, for passing in lots of bounding boxes and testing that things behave well over time, etc.

    # Kind of loose test where I pass in a series of a single bounding box moving around, and I check that
    # the smoothed bb follows.
    # If you want to get fancy, you could plot the box moving over time.
    def test_track_single_bb_over_time(self):
        # Check that, to start, the smooth bb is None
        assert self.cinematic_controller.smoothed_bounding_box is None
        previous_smoothed_bb = None
        # The idea is to create a bounding box that moves from bottom left to upper right.
        num_bbs = 25
        x_step_size = 1.0
        slope_of_movement = 0.5
        fixed_dimensions = (10, 10)
        for bb_index in range(num_bbs):
            new_x = bb_index * x_step_size
            new_y = new_x * slope_of_movement
            new_bb = BoundingBox(fixed_dimensions, (new_x, new_y))
            self.cinematic_controller.update_latest_bbs([new_bb])
            if previous_smoothed_bb is not None:
                # Check that the smoothed bb has moved up and to the right, and at the correct angle.
                new_smoothed_bb = self.cinematic_controller.smoothed_bounding_box
                new_smoothed_bb_centroid = new_smoothed_bb.get_centroid()
                x_diff = new_smoothed_bb_centroid[0] - previous_smoothed_bb.get_centroid()[0]
                y_diff = new_smoothed_bb_centroid[1] - previous_smoothed_bb.get_centroid()[1]
                assert x_diff > 0
                assert y_diff > 0
                assert y_diff == x_diff * slope_of_movement

    # A lot like test_track_single_bb_over_time, but this time add extra small bbs that show up
    # in random locations. The cinematic controller should ignore these extra bbs and keep following
    # the main one.
    def test_track_single_bb_over_time_with_noise(self):
        # Check that, to start, the smooth bb is None
        assert self.cinematic_controller.smoothed_bounding_box is None
        previous_smoothed_bb = None
        # The idea is to create a bounding box that moves from bottom left to upper right.
        num_bbs = 25
        x_step_size = 1.0
        slope_of_movement = 0.5
        fixed_dimensions = (10, 10)
        for bb_index in range(num_bbs):
            new_x = bb_index * x_step_size
            new_y = new_x * slope_of_movement
            new_bb = BoundingBox(fixed_dimensions, (new_x, new_y))
            # Now create a "distractor" bb a little bit out of the way and smaller
            distractor_x = new_x + 2.5  # These are arbitrary choices. Feel free to edit.
            distractor_y = new_y - 0.5
            distractor_dimensions = (fixed_dimensions[0] * 0.5, fixed_dimensions[1] * 0.5)
            distractor_bb = BoundingBox(distractor_dimensions, (distractor_x, distractor_y))
            # Note how the distractor_bb and new_bb are both passed in.
            self.cinematic_controller.update_latest_bbs([new_bb, distractor_bb])
            if previous_smoothed_bb is not None:
                # Check that the smoothed bb has moved up and to the right, and at the correct angle.
                new_smoothed_bb = self.cinematic_controller.smoothed_bounding_box
                new_smoothed_bb_centroid = new_smoothed_bb.get_centroid()
                x_diff = new_smoothed_bb_centroid[0] - previous_smoothed_bb.get_centroid()[0]
                y_diff = new_smoothed_bb_centroid[1] - previous_smoothed_bb.get_centroid()[1]
                assert x_diff > 0
                assert y_diff > 0
                assert y_diff == x_diff * slope_of_movement

    # Test that the drone backs up if it's too close to the bounding box.
    def test_generate_waypoints_too_close(self):
        # Give a bounding box for now that is too close (but is centered correctly)
        target_bb = self.waypoint_generator.get_target_bb()
        target_dimensions = target_bb.get_dimensions()
        larger_bb_dimensions = (target_dimensions[0] * 2, target_dimensions[1] * 2)
        larger_bb = BoundingBox(larger_bb_dimensions, target_bb.get_centroid())
        self.cinematic_controller.update_latest_bbs([larger_bb])
        self.cinematic_controller.update_latest_drone_state(origin_drone_state)
        self.cinematic_controller.set_waypoint_generator(FixedBBWaypointGenerator())

        # Now ask for waypoints, which should guide the drone to back up to make the box smaller.
        waypoints = self.cinematic_controller.generate_waypoints()
        assert len(waypoints) == 1
        waypoint = waypoints[0]
        assert waypoint.get_position()[0] < 0  # X component is now negative
        assert waypoint.get_position()[1] == 0
        assert waypoint.get_position()[2] == 0
        assert waypoint.get_attitude() == (0, 0, 0)

    # Test that the drone goes forward if it's too far from the bounding box.
    def test_generate_waypoints_too_far(self):
        # Give a bounding box for now that is too far (but is centered correctly)
        target_bb = self.waypoint_generator.get_target_bb()
        target_dimensions = target_bb.get_dimensions()
        larger_bb_dimensions = (target_dimensions[0] * 0.5, target_dimensions[1] * 0.5)
        larger_bb = BoundingBox(larger_bb_dimensions, target_bb.get_centroid())
        self.cinematic_controller.update_latest_bbs([larger_bb])
        self.cinematic_controller.update_latest_drone_state(origin_drone_state)
        self.cinematic_controller.set_waypoint_generator(FixedBBWaypointGenerator())

        # Now ask for waypoints, which should guide the drone to go forward to make the box bigger
        waypoints = self.cinematic_controller.generate_waypoints()
        assert len(waypoints) == 1
        waypoint = waypoints[0]
        assert waypoint.get_position()[0] > 0  # X component is now positive because the drone flew forward along x.
        assert waypoint.get_position()[1] == 0
        assert waypoint.get_position()[2] == 0
        assert waypoint.get_attitude() == (0, 0, 0)

    # Test that the drone turns left if the bounding box is to the left of the center
    def test_generate_waypoints_bb_to_left(self):
        target_bb = self.waypoint_generator.get_target_bb()
        target_centroid = target_bb.get_centroid()
        left_bb_centroid = (target_centroid[0] - 10, target_centroid[1])
        left_bb = BoundingBox(target_bb.get_dimensions(), left_bb_centroid)
        self.cinematic_controller.update_latest_bbs([left_bb])
        self.cinematic_controller.update_latest_drone_state(origin_drone_state)
        self.cinematic_controller.set_waypoint_generator(FixedBBWaypointGenerator())

        # Now ask for waypoints, which should guide the drone to turn left to center the bounding box.
        waypoints = self.cinematic_controller.generate_waypoints()
        assert len(waypoints) == 1
        waypoint = waypoints[0]
        assert waypoint.get_position() == (0, 0, 0)
        assert waypoint.get_attitude()[0] == 0
        assert waypoint.get_attitude()[1] == 0
        assert waypoint.get_attitude()[2] > 0

    # Test that the drone turns right if the bounding box is to the right of the center
    def test_generate_waypoints_bb_to_right(self):
        target_bb = self.waypoint_generator.get_target_bb()
        target_centroid = target_bb.get_centroid()
        right_bb_centroid = (target_centroid[0] + 10, target_centroid[1])
        right_bb = BoundingBox(target_bb.get_dimensions(), right_bb_centroid)
        self.cinematic_controller.update_latest_bbs([right_bb])
        self.cinematic_controller.update_latest_drone_state(origin_drone_state)
        self.cinematic_controller.set_waypoint_generator(FixedBBWaypointGenerator())

        # Now ask for waypoints, which should guide the drone to turn right to center the bounding box.
        waypoints = self.cinematic_controller.generate_waypoints()
        assert len(waypoints) == 1
        waypoint = waypoints[0]
        assert waypoint.get_position() == (0, 0, 0)
        assert waypoint.get_attitude()[0] == 0
        assert waypoint.get_attitude()[1] == 0
        assert waypoint.get_attitude()[2] < 0

    # Test that the drone goes up if the bounding box is too high
    def test_generate_waypoints_bb_too_high(self):
        target_bb = self.waypoint_generator.get_target_bb()
        target_centroid = target_bb.get_centroid()
        high_bb_centroid = (target_centroid[0], target_centroid[1] + 10)
        high_bb = BoundingBox(target_bb.get_dimensions(), high_bb_centroid)
        self.cinematic_controller.update_latest_bbs([high_bb])
        self.cinematic_controller.update_latest_drone_state(origin_drone_state)
        self.cinematic_controller.set_waypoint_generator(FixedBBWaypointGenerator())

        # Now ask for waypoints, which should guide the drone to fly up
        waypoints = self.cinematic_controller.generate_waypoints()
        assert len(waypoints) == 1
        waypoint = waypoints[0]
        assert waypoint.get_position()[0] == 0
        assert waypoint.get_position()[1] == 0
        assert waypoint.get_position()[2] > 0  # Fly up.
        assert waypoint.get_attitude() == (0, 0, 0)

    # Test that the drone goes down if the bounding box is too low
    def test_generate_waypoints_bb_too_low(self):
        target_bb = self.waypoint_generator.get_target_bb()
        target_centroid = target_bb.get_centroid()
        low_bb_centroid = (target_centroid[0], target_centroid[1] - 10)
        low_bb = BoundingBox(target_bb.get_dimensions(), low_bb_centroid)
        self.cinematic_controller.update_latest_bbs([low_bb])
        self.cinematic_controller.update_latest_drone_state(origin_drone_state)
        self.cinematic_controller.set_waypoint_generator(FixedBBWaypointGenerator())

        # Now ask for waypoints, which should guide the drone to fly down
        waypoints = self.cinematic_controller.generate_waypoints()
        assert len(waypoints) == 1
        waypoint = waypoints[0]
        assert waypoint.get_position()[0] == 0
        assert waypoint.get_position()[1] == 0
        assert waypoint.get_position()[2] < 0  # Fly down.
        assert waypoint.get_attitude() == (0, 0, 0)

    # Test that, on startup, if no bounding box is given to the drone, it just hovers.
    def test_hover_if_no_bb_at_start(self):
        self.cinematic_controller.update_latest_bbs(None)
        # Check that smoothed and latest have been updated to be None
        latest_bb = self.cinematic_controller.latest_bounding_box
        smoothed_bb = self.cinematic_controller.smoothed_bounding_box
        assert latest_bb is None
        assert smoothed_bb is None
        self.cinematic_controller.update_latest_drone_state(origin_drone_state)
        self.cinematic_controller.set_waypoint_generator(FixedBBWaypointGenerator())

        # Now check that the drone is told to stay in place.
        waypoints = self.cinematic_controller.generate_waypoints()
        assert len(waypoints) == 1
        waypoint = waypoints[0]
        assert waypoint == origin_drone_state

    # Test that, after the the drone has seen a bounding box, if the bounding box disappears, the drone
    # stops and hovers.
    def test_hover_if_no_bb_in_the_middle(self):
        self.cinematic_controller.update_latest_bbs([bb_10_10_0_0])
        self.cinematic_controller.update_latest_drone_state(origin_drone_state)
        self.cinematic_controller.set_waypoint_generator(FixedBBWaypointGenerator())

        # Check that the drone should start flying somewhere
        waypoints = self.cinematic_controller.generate_waypoints()
        assert len(waypoints) == 1
        first_waypoint = waypoints[0]
        assert first_waypoint != origin_drone_state

        # Now send in no bounding boxes
        self.cinematic_controller.update_latest_bbs(None)
        # Now check that the drone is told to continue with the pre-calculated waypoints.
        waypoints = self.cinematic_controller.generate_waypoints()
        assert len(waypoints) == 1
        waypoint = waypoints[0]
        assert waypoint == first_waypoint

    # Test that the ngon waypoint generator can make squares.
    def test_generate_square_waypoints(self):
        self.cinematic_controller.update_latest_drone_state(origin_drone_state)
        self.cinematic_controller.update_latest_bbs([bb_10_10_0_0])
        self.cinematic_controller.set_waypoint_generator(NGonWaypointGenerator(n=4, radius=1))

        # Now ask for waypoints, which should describe a square around the point (1, 0)
        waypoints = self.cinematic_controller.generate_waypoints()
        assert len(waypoints) == 4  # 4 points
        waypoint1, waypoint2, waypoint3, waypoint4 = waypoints
        # Check that the positions are correct
        assert TestCinematicController.are_points_close(waypoint1.get_position(), (1, -1, 0))
        assert TestCinematicController.are_points_close(waypoint2.get_position(), (2, 0, 0))
        assert TestCinematicController.are_points_close(waypoint3.get_position(), (1, 1, 0))
        assert TestCinematicController.are_points_close(waypoint4.get_position(), (0, 0, 0))
        # Check that the drone yaw gets updated, too
        epsilon = 0.01  # How much mathematical error is allowed
        assert abs(waypoint1.get_attitude()[2] - math.pi / 2) < epsilon
        assert abs(waypoint2.get_attitude()[2] - math.pi) < epsilon
        assert abs(waypoint3.get_attitude()[2] - 3 * math.pi / 2) < epsilon
        assert abs(waypoint4.get_attitude()[2] - 2 * math.pi) < epsilon

    # Test that the ngon waypoint generator can make squares around a moving target
    def test_generate_square_waypoints_around_prediction(self):
        self.cinematic_controller.person_predictor = PolynomialPredictor(weight_decay_factor=2)
        self.cinematic_controller.update_latest_drone_state(origin_drone_state)
        self.cinematic_controller.update_latest_bbs([BoundingBox((10, 10), (0, 0))])
        time.sleep(1)
        self.cinematic_controller.update_latest_bbs([BoundingBox((9, 9), (0, 0))])
        time.sleep(1)
        self.cinematic_controller.update_latest_bbs([BoundingBox((8, 8), (0, 0))])
        time.sleep(1)
        self.cinematic_controller.update_latest_bbs([BoundingBox((7, 7), (0, 0))])
        time.sleep(1)
        self.cinematic_controller.update_latest_bbs([BoundingBox((6, 6), (0, 0))])
        self.cinematic_controller.set_waypoint_generator(NGonWaypointGenerator(n=4, radius=1))

        # Now ask for waypoints, which should describe a square around moving person
        waypoints = self.cinematic_controller.generate_waypoints()
        assert len(waypoints) == 4  # 4 points
        waypoint1, waypoint2, waypoint3, waypoint4 = waypoints
        # Check that the positions are correct
        allowed_error = 0.01
        assert waypoint1.get_position()[0] > 0
        assert abs(waypoint1.get_position()[1] + 1) < allowed_error
        assert waypoint2.get_position()[0] > waypoint1.get_position()[0]
        assert abs(waypoint2.get_position()[1]) < allowed_error
        assert waypoint3.get_position()[0] < waypoint2.get_position()[0]
        assert abs(waypoint3.get_position()[1] - 1) < allowed_error
        assert waypoint4.get_position()[0] < waypoint3.get_position()[0]
        assert abs(waypoint4.get_position()[1]) < allowed_error
        # Check that the drone yaw gets updated, too
        epsilon = 0.01  # How much mathematical error is allowed
        assert abs(waypoint1.get_attitude()[2] - math.pi / 2) < epsilon
        assert abs(waypoint2.get_attitude()[2] - math.pi) < epsilon
        assert abs(waypoint3.get_attitude()[2] - 3 * math.pi / 2) < epsilon
        assert abs(waypoint4.get_attitude()[2] - 2 * math.pi) < epsilon

    @staticmethod
    def are_points_close(point1, point2):
        epsilon = 0.001  # How close is close enough
        return abs(point1[0] - point2[0]) < epsilon and \
               abs(point1[1] - point2[1]) < epsilon and \
               abs(point1[2] - point2[2]) < epsilon

    # Test that if the bounding box is to the right, the drone yaws right.
    def test_generate_yaw_waypoints_right(self):
        yaw_waypoint_generator = YawWaypointGenerator()
        self.cinematic_controller.set_waypoint_generator(yaw_waypoint_generator)
        target_x = yaw_waypoint_generator.get_target_centroid_x()
        target_centroid = (target_x, 0)
        right_bb_centroid = (target_centroid[0] + 10, target_centroid[1])
        right_bb = BoundingBox((10, 10), right_bb_centroid)
        self.cinematic_controller.update_latest_bbs([right_bb])
        self.cinematic_controller.update_latest_drone_state(origin_drone_state)

        # Now ask for waypoints, which should guide the drone to turn right to center the bounding box.
        waypoints = self.cinematic_controller.generate_waypoints()
        assert len(waypoints) == 1
        waypoint = waypoints[0]
        assert waypoint.get_position() == (0, 0, 0)
        assert waypoint.get_attitude()[0] == 0
        assert waypoint.get_attitude()[1] == 0
        assert waypoint.get_attitude()[2] < 0

    # Test that if the bounding box is to the left, the drone yaws left.
    def test_generate_yaw_waypoints_left(self):
        yaw_waypoint_generator = YawWaypointGenerator()
        self.cinematic_controller.set_waypoint_generator(yaw_waypoint_generator)
        target_x = yaw_waypoint_generator.get_target_centroid_x()
        target_centroid = (target_x, 0)
        left_bb_centroid = (target_centroid[0] - 10, target_centroid[1])
        left_bb = BoundingBox((10, 10), left_bb_centroid)
        self.cinematic_controller.update_latest_bbs([left_bb])
        self.cinematic_controller.update_latest_drone_state(origin_drone_state)

        # Now ask for waypoints, which should guide the drone to turn left to center the bounding box.
        waypoints = self.cinematic_controller.generate_waypoints()
        assert len(waypoints) == 1
        waypoint = waypoints[0]
        assert waypoint.get_position() == (0, 0, 0)
        assert waypoint.get_attitude()[0] == 0
        assert waypoint.get_attitude()[1] == 0
        assert waypoint.get_attitude()[2] > 0

    # Test that visualization plot can be generated and saved properly with simple inputs
    def test_waypoint_visualization_with_ngon_generator(self):
        # initialize the drone state
        waypoint_generator = NGonWaypointGenerator(n=6, radius=4)
        gamma = 0.9
        cinematic_controller = CinematicController(waypoint_generator=waypoint_generator, bb_filter_gamma=gamma)
        cinematic_controller.update_latest_drone_state(origin_drone_state)
        cinematic_controller.update_latest_bbs([bb_10_10_0_0])

        # initialize the drone waypoints
        waypoints = cinematic_controller.generate_waypoints()

        # initialize the person state
        person_state = PersonState(4, 0)

        # initialize the environment
        environment = Environment()
        environment.add_obstacles([[(6, 0), (4, 2), (6, 2)],
                                   [(7, -4), (8, -3), (9, -3), (9, -5), (8, -3.5)],
                                   [(-5, -2), (0, -5), (0, -2)]])

        # initialize visualization object and plot
        viz = WaypointVisualization(waypoints, [person_state], environment)
        viz.plot('plots/test_waypoint_visualization_with_ngon_generator.png')

        # check that file was saved properly
        assert os.path.isfile('plots/test_waypoint_visualization_with_ngon_generator.png')

    # Test that the controller moves onto the next waypoint appropriately
    def test_next_waypoint_with_ngon_generator(self):
        # initialize the drone state
        self.cinematic_controller.update_latest_drone_state(origin_drone_state)
        self.cinematic_controller.set_waypoint_generator(NGonWaypointGenerator(n=4, radius=1))

        # generate waypoints
        waypoints = self.cinematic_controller.generate_waypoints()
        assert len(waypoints) == 4
        waypoint1_old, waypoint2_old, waypoint3_old, waypoint4_old = waypoints

        # artificially update state to test waypoint update
        self.cinematic_controller.latest_drone_state.x = waypoint1_old.x
        self.cinematic_controller.latest_drone_state.y = waypoint1_old.y
        self.cinematic_controller.latest_drone_state.z = waypoint1_old.z

        # Regenerate waypoints
        waypoints = self.cinematic_controller.generate_waypoints()
        # The first waypoint should be deleted so the length should now be 3
        assert len(waypoints) == 3
        waypoint1_new, waypoint2_new, waypoint3_new = waypoints
        # The new next waypoint should be the same as the old 2nd waypoint
        assert waypoint1_new == waypoint2_old

    # Test visualization for ngon points around a predicted humans path
    def test_waypoint_visualization_with_ngon_generator_predicted_human(self):
        # initialize the drone state
        waypoint_generator = NGonWaypointGenerator(n=6, radius=4)
        gamma = 0.9
        cinematic_controller = CinematicController(waypoint_generator=waypoint_generator, bb_filter_gamma=gamma)
        cinematic_controller.update_latest_drone_state(DroneState(x=2, y=-3))
        cinematic_controller.person_predictor = PolynomialPredictor(poly_degree=2, weight_decay_factor=1)
        cinematic_controller.update_latest_bbs([BoundingBox((10, 10), (0, 0))])
        time.sleep(.001)
        cinematic_controller.update_latest_bbs([BoundingBox((5, 5), (50, 0))])
        time.sleep(.001)
        cinematic_controller.update_latest_bbs([BoundingBox((2, 2), (50, 0))])
        time.sleep(.001)
        cinematic_controller.update_latest_bbs([BoundingBox((1, 1), (50, 0))])

        # initialize the drone waypoints
        waypoints = cinematic_controller.generate_waypoints()

        # initialize the person state
        person_states = cinematic_controller.person_predictor.predict_next_person_state([2, 4, 6, 8, 10, 15, 20, 25, 30])

        # initialize the environment
        environment = Environment()
        environment.add_obstacles([[(6, 0), (4, 2), (6, 2)],
                                   [(8, -4), (9, -3), (10, -3), (10, -5), (9, -3.5)],
                                   [(-6, -2), (-1, -5), (-1, -2)]])

        # initialize visualization object and plot
        viz = WaypointVisualization(waypoints, person_states, environment)
        viz.plot('plots/moving_target.png')


if __name__ == '__main__':
    unittest.main()
