"""
Generator of smooth intermediate goals
Input: List of (goal, time)
Output: Commands sent to the drone to follow the intermediate goals
"""

import numpy as np


def smooth_gen(list_goals):
	for goal, t in list_goals:
		# assume goal is a class with attributes goal.pos, goal.vel...
		...
		send_commands(goal)


def send_commands(goal):
	pass
