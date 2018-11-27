import numpy as np
import matplotlib.pyplot as plt
import time
from utils.person_state import PersonState


# Class used for predicting where a person will go.
# Uses a sliding window for some fixed number of PersonStates in the past.
# Fits a polynomial to project into the future.
class PolynomialPredictor:
    def __init__(self, num_states_to_track=4, poly_degree=2, weight_decay_factor=1):
        self.person_states = []
        self.num_states_to_track = num_states_to_track
        self.poly_degree = poly_degree
        self.poly = None
        self.weight_decay_factor = weight_decay_factor  # Some crazy stuff I'm doing about weighting the poly.

    def add_person_state(self, person_state):
        current_time = time.time()  # Gets the current time in seconds
        self.person_states.append((person_state, current_time))
        if len(self.person_states) > self.num_states_to_track:
            del self.person_states[0]  # Get rid of most stale person state.

        # If there are enough states to fit a predictor, do so.
        if len(self.person_states) >= self.poly_degree:
            x_data = [state_time_tuple[0].x for state_time_tuple in self.person_states]
            y_data = [state_time_tuple[0].y for state_time_tuple in self.person_states]
            weighted_x_data = []
            weighted_y_data = []
            for i, x in enumerate(x_data):
                weight_factor = self.weight_decay_factor ** i
                y = y_data[i]
                for j in range(weight_factor):
                    weighted_x_data.append(x)
                    weighted_y_data.append(y)
            quadratic_coeffs = np.polyfit(weighted_x_data, weighted_y_data, self.poly_degree)
            self.poly = np.poly1d(quadratic_coeffs)

    # Accept as input a list of seconds to project into the future (e.g. [10, 15] means
    # that this method should return predictions for the drone 10 and 15 seconds from
    # now).
    def predict_next_person_state(self, time_deltas_to_predict):
        num_datapoints = len(self.person_states)
        if num_datapoints == 0:
            print("ERROR: cannot predict if not given any past data")
            return None
        if num_datapoints == 1:
            print("WARNING: asked to predict but only given a single past datapoint.")
            return [self.person_states[0][0] for _ in time_deltas_to_predict]

        # Estimate the distance covered so we can back out the velocity
        distance_covered = 0
        for i in range(len(self.person_states) - 1):
            current_state = self.person_states[i][0]
            next_state = self.person_states[i + 1][0]
            distance_covered += np.sqrt((current_state.x - next_state.x)**2 + (current_state.y - next_state.y)**2)
        oldest_state, oldest_time = self.person_states[0]
        newest_state, newest_time = self.person_states[-1]
        projected_speed = distance_covered / (newest_time - oldest_time)

        # Project for each of the requested times
        projections = []
        step_size_x = 0.00001  # The smaller the better resolution, but also slower computationally.
        for time_delta in time_deltas_to_predict:
            distance_to_cover = time_delta * projected_speed
            # Inch forward along the curve
            previous_x = newest_state.x
            previous_y = newest_state.y
            projected_x = previous_x + step_size_x
            projected_y = self.poly(projected_x)
            distance_projected = 0
            # Obviously, this is a numerical method that's inexact, and what I'm doing will always overshoot
            # instead of finding an unbiased estimate, and it's computationally inefficient because I'm
            # stepping forward linearly instead of growing step size exponentially. It's super easy to implement
            # and understand, at least.
            while distance_projected < distance_to_cover:
                projected_x = previous_x + step_size_x
                projected_y = self.poly(projected_x)
                distance_projected += np.sqrt(step_size_x**2 + (projected_y - previous_y)**2)
                previous_x = projected_x
                previous_y = projected_y
            print("Overshot distance by:", distance_projected - distance_to_cover)
            projections.append(PersonState(projected_x, projected_y))
        return projections

    def plot_projections(self, projections):
        x_data = [state_time_tuple[0].x for state_time_tuple in self.person_states]
        y_data = [state_time_tuple[0].y for state_time_tuple in self.person_states]

        projected_x = [projection.x for projection in projections]
        projected_y = [projection.y for projection in projections]
        smooth_projected_x = np.linspace(min(x_data + projected_x), max(x_data + projected_x), 100)
        smooth_projected_y = [self.poly(x) for x in smooth_projected_x]

        _ = plt.plot(x_data, y_data, '.', smooth_projected_x, smooth_projected_y, '-',
                     projected_x, projected_y, 'o')

        plt.show()
