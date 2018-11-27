#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Generator of smooth intermediate goals
Input: List of (goal, time)
Output: Commands sent to the drone to follow the intermediate goals
"""

import numpy as np
from scipy.interpolate import interp2d
#from utils.drone_state import DroneState
from move_commands import *
import sys


def smooth_gen(list_states):
    # list of drone states
    goal = list_states[0]
    ...



if __name__ == "__main__":
    from pyparrot.Minidrone import Mambo

    # If you are using BLE: you will need to change this to the address of YOUR mambo
    # if you are using Wifi, this can be ignored
    mamboAddr = "E0:14:F1:84:3D:CA"

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

                move_forward(mambo, 20, 1)
                move_back(mambo, 20, 1)
                move_left(mambo, 20, 1)
                move_right(mambo, 20, 1)
                move_up(mambo, 20, 1)
                move_down(mambo, 20, 1)
                yaw(mambo, -90)


        except Exception as e: 
            print("Error:", e)

        print("landing now. Flying state is %s" % mambo.sensors.flying_state)
        mambo.safe_land(5)
        mambo.smart_sleep(5)
        print("disconnect")
        mambo.disconnect()
