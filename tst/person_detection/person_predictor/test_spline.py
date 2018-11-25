import time
import unittest
from person_detection.person_predictor.spline_predictor import SplinePredictor
from utils.person_state import PersonState

# Define a few helpful variables common across a few tests
origin_person_state = PersonState(0, 0)
second_intervals = [1, 2, 3, 4, 5, 6]


class TestSpline(unittest.TestCase):
    # Before every test, reset the SplinePredictor object that will be tested.
    def setUp(self):
        self.spline_predictor = SplinePredictor(bc_type='natural', extrapolation_type=None)

    def test_one_state_given(self):
        # SETUP
        self.spline_predictor.add_person_state(origin_person_state)

        # EXECUTE
        predicted_states = self.spline_predictor.predict_next_person_state(second_intervals)

        # VERIFY
        # Given only a single state, the best prediction over any time horizon is to stay in the same place.
        assert len(predicted_states) == len(second_intervals)
        for predicted_state in predicted_states:
            assert predicted_state == origin_person_state

    def test_cubic_standard(self):
        # SETUP
        # All default (not-a-knot and normal extrapolation based on last interval)
        self.spline_predictor = SplinePredictor()
        # Feed in states going through (-1, 1), (0, 0), (1, 1), (2, 1), (3, 1)
        self.spline_predictor.add_person_state(PersonState(-1, 1))
        time.sleep(1)
        self.spline_predictor.add_person_state(origin_person_state)
        time.sleep(1)
        self.spline_predictor.add_person_state(PersonState(1, 1))
        time.sleep(1)
        self.spline_predictor.add_person_state(PersonState(2, 1))
        time.sleep(1)
        self.spline_predictor.add_person_state(PersonState(3, 1))

        # EXECUTE
        predicted_states = self.spline_predictor.predict_next_person_state(second_intervals)

        # VERIFY
        assert len(predicted_states) == len(second_intervals)
        prev_x = 1
        for predicted_state in predicted_states:
            # Very loose. Fix me?
            assert predicted_state.x > prev_x
            prev_x = predicted_state.x

        # Plot things for visual inspection
        self.spline_predictor.plot_projections(predicted_states)

    def test_cubic_natural_end_points(self):
        # SETUP
        self.spline_predictor = SplinePredictor(bc_type='natural')
        # Feed in states going through (-1, 1), (0, 0), (1, 1), (2, 1), (3, 1)
        self.spline_predictor.add_person_state(PersonState(-1, 1))
        time.sleep(1)
        self.spline_predictor.add_person_state(origin_person_state)
        time.sleep(1)
        self.spline_predictor.add_person_state(PersonState(1, 1))
        time.sleep(1)
        self.spline_predictor.add_person_state(PersonState(2, 1))
        time.sleep(1)
        self.spline_predictor.add_person_state(PersonState(3, 1))

        # EXECUTE
        predicted_states = self.spline_predictor.predict_next_person_state(second_intervals)

        # VERIFY
        assert len(predicted_states) == len(second_intervals)
        prev_x = 1
        for predicted_state in predicted_states:
            # Very loose. Fix me?
            assert predicted_state.x > prev_x
            prev_x = predicted_state.x

        # Plot things for visual inspection
        self.spline_predictor.plot_projections(predicted_states)

    # Use periodic extrapolation. Almost certainly dumb.
    def test_cubic_periodic_extrapolation(self):
        # SETUP
        self.spline_predictor = SplinePredictor(bc_type='natural', extrapolation_type='periodic')
        # Feed in states going through (-1, 1), (0, 0), (1, 1), (2, 1), (3, 1)
        self.spline_predictor.add_person_state(PersonState(-1, 1))
        time.sleep(1)
        self.spline_predictor.add_person_state(origin_person_state)
        time.sleep(1)
        self.spline_predictor.add_person_state(PersonState(1, 1))
        time.sleep(1)
        self.spline_predictor.add_person_state(PersonState(2, 1))
        time.sleep(1)
        self.spline_predictor.add_person_state(PersonState(3, 1))

        # EXECUTE
        predicted_states = self.spline_predictor.predict_next_person_state(second_intervals)

        # VERIFY
        assert len(predicted_states) == len(second_intervals)
        prev_x = 1
        for predicted_state in predicted_states:
            # Very loose. Fix me?
            assert predicted_state.x > prev_x
            prev_x = predicted_state.x

        # Plot things for visual inspection
        self.spline_predictor.plot_projections(predicted_states)

    def test_cubic_many_datapoints(self):
        # SETUP
        self.spline_predictor = SplinePredictor(num_states_to_track=100, bc_type='natural')
        # Feed in tons of states.
        self.spline_predictor.add_person_state(PersonState(-1, 1))
        time.sleep(1)
        self.spline_predictor.add_person_state(origin_person_state)
        time.sleep(1)
        self.spline_predictor.add_person_state(PersonState(1, 1))
        time.sleep(1)
        self.spline_predictor.add_person_state(PersonState(2, 1))
        time.sleep(1)
        self.spline_predictor.add_person_state(PersonState(3, 1))
        time.sleep(1)
        self.spline_predictor.add_person_state(PersonState(4, 2))
        time.sleep(1)
        self.spline_predictor.add_person_state(PersonState(5, 1))

        # EXECUTE
        predicted_states = self.spline_predictor.predict_next_person_state(second_intervals)

        # VERIFY
        assert len(predicted_states) == len(second_intervals)
        prev_x = 1
        for predicted_state in predicted_states:
            # Very loose. Fix me?
            assert predicted_state.x > prev_x
            prev_x = predicted_state.x

        # Plot things for visual inspection
        self.spline_predictor.plot_projections(predicted_states)
