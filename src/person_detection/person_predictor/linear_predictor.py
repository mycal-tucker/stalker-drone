import time
from utils.person_state import PersonState


# Class used for predicting where a person will go.
# Uses a sliding window for some fixed number of PersonStates in the past
# to linearly project where the person will be in the future. Obviously,
# linear projection is very crude, so other predictors should eventually
# replace this object.
class LinearPredictor:
    def __init__(self, num_states_to_track=5):
        self.person_states = []
        self.num_states_to_track = num_states_to_track

    def add_person_state(self, person_state):
        current_time = time.time()  # Gets the current time in seconds
        self.person_states.append((person_state, current_time))
        if len(self.person_states) > self.num_states_to_track:
            del self.person_states[0]  # Get rid of most stale person state.

    # Accept as input a list of seconds to project into the future (e.g. [10, 15] means
    # that this method should return predictions for the drone 10 and 15 seconds from
    # now).
    def predict_next_person_state(self, time_deltas_to_predict):
        # Translate time_deltas into global times
        current_time = time.time()
        times_to_predict = [time_delta + current_time for time_delta in time_deltas_to_predict]

        num_datapoints = len(self.person_states)
        if num_datapoints == 0:
            print("ERROR: cannot predict if not given any past data")
            return None
        if num_datapoints == 1:
            print("WARNING: asked to predict but only given a single past datapoint.")
            return [self.person_states[0][0] for _ in times_to_predict]
        oldest_state, oldest_time = self.person_states[0]
        newest_state, newest_time = self.person_states[-1]

        delta_x_per_sec = (newest_state.x - oldest_state.x) / (newest_time - oldest_time)
        delta_y_per_sec = (newest_state.y - oldest_state.y) / (newest_time - oldest_time)

        # Project for each of the requested times
        projections = []
        for time_to_predict in times_to_predict:
            x = newest_state.x + delta_x_per_sec * (time_to_predict - newest_time)
            y = newest_state.y + delta_y_per_sec * (time_to_predict - newest_time)
            projections.append(PersonState(x, y))
        return projections
