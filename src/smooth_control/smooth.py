"""
Generator of smooth intermediate goals
Input: List of (goal, time)
Output: Commands sent to the drone to follow the intermediate goals
"""

import numpy as np
from scipy.interpolate import interp2d
#from utils.drone_state import DroneState


def smooth_gen(list_states):
    # list of drone states
    goal = list_states[0]
    send_commands(goal)


def send_commands(command):
    print(command)
    pass
