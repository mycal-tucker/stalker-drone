#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Generator of smooth intermediate goals
Input: List of (goal, time)
Output: Commands sent to the drone to follow the intermediate goals
"""

import numpy as np
from scipy.interpolate import interp2d
from utils.drone_state import DroneState
from move_commands import *
import sys


def test_state_yaw_backward(mambo, dur):
    d1 = DroneState(z=-1)
    d2 = DroneState(y = 0.33, x = 0.3, z = -1.16, yaw = -25)
    d3 = DroneState(y = 0.66, x = 0.3, z = -1.33, yaw = -50)
    drone_states = [d1, d2, d3]

    cinematic_waypoints = [DroneState(y = 1, z = -1.5, yaw = -75)]

    for drone_state in drone_states:
        smooth_gen(drone_state, cinematic_waypoints, dur)
        mambo.smart_sleep(dur)



def test_state_yaw_forward(mambo, dur):
    d1 = DroneState()
    d2 = DroneState(y = -0.33, x = 0.1, yaw = 30)
    d3 = DroneState(y = -0.66, x = 0.1, yaw = 60)
    #d4 = DroneState(y = -1, yaw = 90)
    drone_states = [d1, d2, d3]

    cinematic_waypoints = [DroneState(y = -1, yaw = 90)]

    for drone_state in drone_states:
        smooth_gen(drone_state, cinematic_waypoints, dur)
        mambo.smart_sleep(dur)


def test_state_forward(mambo, dur):
    d1 = DroneState()
    d2 = DroneState(y = -0.5)
    d3 = DroneState(y = -1)
    drone_states = [d1, d2, d3]

    cinematic_waypoints = [DroneState(y = -1)]

    for drone_state in drone_states:
        smooth_gen(drone_state, cinematic_waypoints, dur)
        mambo.smart_sleep(dur)


def test_simple(mambo, dur):
    move(mambo, dx=-0.1, dy=-0.5, dz=0, dyaw=90, duration=dur)
    move_forward(mambo, 20, dur)
    move_backward(mambo, 20, dur)
    move_left(mambo, 20, dur)
    move_right(mambo, 20, dur)
    move_up(mambo, 20, dur)
    move_down(mambo, 20, dur)
    yaw(mambo, -90)


def smooth_gen(drone_state, cinematic_waypoints, duration=1):
    # list of drone states
    goal = cinematic_waypoints[0]

    Dx = goal.x - drone_state.x
    Dy = goal.y - drone_state.y
    dz = goal.z - drone_state.z
    dyaw = goal.yaw - drone_state.yaw

    dx = Dx*np.sin(np.deg2rad(-dyaw))
    dy = Dy*np.cos(np.deg2rad(-dyaw))

    move(mambo, dx=dx, dy=dy, dz=dz, dyaw=dyaw, duration=duration)


if __name__ == "__main__":
    from pyparrot.Minidrone import Mambo

    # If you are using BLE: you will need to change this to the address of YOUR mambo
    # if you are using Wifi, this can be ignored
    mamboAddr = "E0:14:B1:35:3D:CB"  # "E0:14:F1:84:3D:CA"

    # make my mambo object
    # remember to set True/False for the wifi depending on if you are using the wifi or the BLE to connect
    mambo = Mambo(mamboAddr, use_wifi=False)

    print("trying to connect")
    success = mambo.connect(num_retries=3)
    print("connected: %s" % success)

    if (success):
        try:
            # get the state information
            print("sleeping")
            mambo.smart_sleep(2)  # sleeps the requested number of seconds but wakes up for notifications
            mambo.ask_for_state_update()
            mambo.smart_sleep(2)

            print("taking off!")
            mambo.safe_takeoff(5)

            if (mambo.sensors.flying_state != "emergency"):
                dur = 1

                test_state_yaw_backward(mambo, dur)
                # test_state_yaw_forward(mambo, dur)
                # test_state_forward(mambo, dur)
                # test_simple(mambo, dur)


        except Exception as e: 
            print("Error:", e)

        print("landing now. Flying state is %s" % mambo.sensors.flying_state)
        mambo.safe_land(5)
        mambo.smart_sleep(5)
        print("disconnect")
        mambo.disconnect()
