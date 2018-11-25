import time
import unittest
from person_detection.person_predictor.linear_predictor import LinearPredictor
from utils.person_state import PersonState


# Define a few helpful variables common across a few tests
origin_person_state = PersonState(0, 0)
x10_person_state = PersonState(10, 0)
ten_second_intervals = [10, 20, 30, 40, 50, 60]


class TestLinearPredictor(unittest.TestCase):
    # Before every test, reset the LinearPredictor object that will be tested.
    def setUp(self):
        self.linear_predictor = LinearPredictor()

    def test_one_state_given(self):
        # SETUP
        self.linear_predictor.add_person_state(origin_person_state)

        # EXECUTE
        predicted_states = self.linear_predictor.predict_next_person_state(ten_second_intervals)

        # VERIFY
        # Given only a single state, the best prediction over any time horizon is to stay in the same place.
        assert len(predicted_states) == len(ten_second_intervals)
        for predicted_state in predicted_states:
            assert predicted_state == origin_person_state

    def test_two_states_given(self):
        # SETUP
        self.linear_predictor.add_person_state(origin_person_state)
        # Wait 1 seconds to give a chance to predict velocity.
        time.sleep(1)
        self.linear_predictor.add_person_state(x10_person_state)

        # EXECUTE
        predicted_states = self.linear_predictor.predict_next_person_state(ten_second_intervals)

        # VERIFY
        # Given two states, should be able to project out into the future along the x axis
        assert len(predicted_states) == len(ten_second_intervals)
        prev_x = 0
        for predicted_state in predicted_states:
            # Because the above sleep isn't exact, this test is a bit loose and just says that at least
            # the prediction should be heading along the x axis in the positive direction.
            assert predicted_state.x > prev_x
            assert predicted_state.y == 0
            prev_x = predicted_state.x


if __name__ == '__main__':
    unittest.main()
