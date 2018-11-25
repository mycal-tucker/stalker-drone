import time
import unittest
from person_detection.person_predictor.polynomial_predictor import PolynomialPredictor
from utils.person_state import PersonState


# Define a few helpful variables common across a few tests
origin_person_state = PersonState(0, 0)
second_intervals = [1, 2, 3, 4, 5, 6]


class TestPolynomialPredictor(unittest.TestCase):
    # Before every test, reset the PolynomialPredictor object that will be tested.
    def setUp(self):
        self.polynomial_predictor = PolynomialPredictor()

    def test_one_state_given(self):
        # SETUP
        self.polynomial_predictor.add_person_state(origin_person_state)

        # EXECUTE
        predicted_states = self.polynomial_predictor.predict_next_person_state(second_intervals)

        # VERIFY
        # Given only a single state, the best prediction over any time horizon is to stay in the same place.
        assert len(predicted_states) == len(second_intervals)
        for predicted_state in predicted_states:
            assert predicted_state == origin_person_state

    def test_three_states_given(self):
        # SETUP
        # Feed in states that match a parabola going through (-1, 1), (0, 0) and (1, 1)
        self.polynomial_predictor.add_person_state(PersonState(-1, 1))
        time.sleep(1)
        self.polynomial_predictor.add_person_state(origin_person_state)
        time.sleep(1)
        self.polynomial_predictor.add_person_state(PersonState(1, 1))

        # EXECUTE
        predicted_states = self.polynomial_predictor.predict_next_person_state(second_intervals)

        # VERIFY
        # Given states provided, should predict somewhere along the parabola.
        assert len(predicted_states) == len(second_intervals)
        prev_x = 1
        prev_y = 1
        for predicted_state in predicted_states:
            # Very loose. Fix me?
            assert predicted_state.x > prev_x
            assert predicted_state.y > prev_y
            prev_x = predicted_state.x
            prev_y = predicted_state.y

        # Plot things for visual inspection
        self.polynomial_predictor.plot_projections(predicted_states)

    def test_cubic(self):
        # SETUP
        # Make it a cubic predictor
        self.polynomial_predictor = PolynomialPredictor(num_states_to_track=5, poly_degree=3)
        # Feed in states that match a parabola going through (-1, 1), (0, 0), (1, 1), (2, 1), (3, 1)
        self.polynomial_predictor.add_person_state(PersonState(-1, 1))
        time.sleep(1)
        self.polynomial_predictor.add_person_state(origin_person_state)
        time.sleep(1)
        self.polynomial_predictor.add_person_state(PersonState(1, 1))
        time.sleep(1)
        self.polynomial_predictor.add_person_state(PersonState(2, 1))
        time.sleep(1)
        self.polynomial_predictor.add_person_state(PersonState(3, 1))

        # EXECUTE
        predicted_states = self.polynomial_predictor.predict_next_person_state(second_intervals)

        # VERIFY
        assert len(predicted_states) == len(second_intervals)
        prev_x = 1
        prev_y = 1
        for predicted_state in predicted_states:
            # Very loose. Fix me?
            assert predicted_state.x > prev_x
            assert predicted_state.y < prev_y
            prev_x = predicted_state.x
            prev_y = predicted_state.y

        # Plot things for visual inspection
        self.polynomial_predictor.plot_projections(predicted_states)

    def test_weights(self):
        # SETUP
        # Override the weight factor for the weighted polynomial predictor (but keep self.pp unchanged)
        weighted_polynomial_predictor = PolynomialPredictor(num_states_to_track=4, poly_degree=2, weight_decay_factor=4)
        # Feed in states that match a parabola going through (-1, 1), (0, 0), (1, 1), (2, 0)
        self.polynomial_predictor.add_person_state(PersonState(-1, 1))
        weighted_polynomial_predictor.add_person_state(PersonState(-1, 1))
        time.sleep(1)
        self.polynomial_predictor.add_person_state(origin_person_state)
        weighted_polynomial_predictor.add_person_state(origin_person_state)
        time.sleep(1)
        self.polynomial_predictor.add_person_state(PersonState(1, 1))
        weighted_polynomial_predictor.add_person_state(PersonState(1, 1))
        time.sleep(1)
        self.polynomial_predictor.add_person_state(PersonState(2, 0))
        weighted_polynomial_predictor.add_person_state(PersonState(2, 0))

        # EXECUTE
        predicted_states = self.polynomial_predictor.predict_next_person_state(second_intervals)
        weighted_predicted_states = weighted_polynomial_predictor.predict_next_person_state(second_intervals)

        # VERIFY
        assert len(predicted_states) == len(second_intervals)
        assert len(weighted_predicted_states) == len(second_intervals)
        prev_x = 2
        prev_y = 0
        weighted_prev_x = 2
        weighted_prev_y = 0
        for i, predicted_state in enumerate(predicted_states):
            weighted_predicted_state = weighted_predicted_states[i]
            # Very loose. Fix me?
            assert predicted_state.x > prev_x
            assert predicted_state.y < prev_y
            prev_x = predicted_state.x
            prev_y = predicted_state.y

            assert weighted_predicted_state.x > weighted_prev_x
            assert weighted_predicted_state.y < weighted_prev_y
            weighted_prev_x = weighted_predicted_state.x
            weighted_prev_y = weighted_predicted_state.y

        # Plot things for visual inspection
        self.polynomial_predictor.plot_projections(predicted_states)
        weighted_polynomial_predictor.plot_projections(weighted_predicted_states)


if __name__ == '__main__':
    unittest.main()
