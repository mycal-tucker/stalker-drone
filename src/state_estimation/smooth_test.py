#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Dec  3 08:05:32 2018

@author: Rosemary
"""

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Generator of smooth intermediate goals
Input: List of (goal, time)
Output: Commands sent to the drone to follow the intermediate goals
"""

import numpy as np
# from pyparrot.drone_state import DroneState
# from pyparrot.move_commands import *
from utils.drone_state import DroneState
from smooth_control.move_commands import *
import sys
from pyparrot.Minidrone import Mambo
from pyparrot.Minidrone import MinidroneSensors
import time


def test_multiple_states(mambo, dur):
    d1 = DroneState()

    cinematic_waypoints = [DroneState(y = -1, z = -1, yaw = 90), DroneState(y = 1, z = 1, yaw = -90)]

    for cinematic_waypoint in cinematic_waypoints:
        cinematic_waypoint = [cinematic_waypoint]
        smooth_gen(d1, cinematic_waypoint, dur)
        mambo.smart_sleep(dur+0.5)

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

    dx = Dx*np.sin(np.deg2rad(-drone_state.yaw))
    dy = Dy*np.cos(np.deg2rad(-drone_state.yaw))

    move(mambo, dx=dx, dy=dy, dz=dz, dyaw=dyaw, duration=duration)
    
def get_velocity(mambo):
    vel_x = mambo.sensors.speed_x
    vel_y = mambo.sensors.speed_y
    vel_z = mambo.sensors.speed_z
        
    return vel_x, vel_y, vel_z

def get_position(mambo,last_x,last_y,last_z,last_time):
    old_pos_z = last_z
    old_pos_x = last_x
    old_pos_y = last_y
    old_time = last_time
    #current_time = time.time()
    current_time = mambo.sensors.speed_ts
    elapsed_time = (current_time - old_time)/1000.0
    pos_x = old_pos_x + (mambo.sensors.speed_x * elapsed_time)
    pos_y = old_pos_y + (mambo.sensors.speed_y * elapsed_time)
    pos_z = old_pos_z + (mambo.sensors.speed_z * elapsed_time)
    
    pos_z_compare = mambo.sensors.altitude
    
    return pos_x, pos_y, pos_z, pos_z_compare, current_time

def get_attitude(mambo):
    q1 = mambo.sensors.quaternion_w
    q2 = mambo.sensors.quaternion_x
    q3 = mambo.sensors.quaternion_y
    q4 = mambo.sensors.quaternion_z
        
    roll, pitch, yaw = MinidroneSensors.quaternion_to_euler_angle(mambo, q1, q2, q3, q4)

    return roll, pitch, yaw

def get_attitude_rate(mambo):
    firstq1 = mambo.sensors.quaternion_w
    firstq2 = mambo.sensors.quaternion_x
    firstq3 = mambo.sensors.quaternion_y
    firstq4 = mambo.sensors.quaternion_z
        
    firstroll, firstpitch, firstyaw = MinidroneSensors.quaternion_to_euler_angle(mambo, firstq1, firstq2, firstq3, firstq4)
        
    mambo.ask_for_state_update()
        
    newq1 = mambo.sensors.quaternion_w
    newq2 = mambo.sensors.quaternion_x
    newq3 = mambo.sensors.quaternion_y
    newq4 = mambo.sensors.quaternion_z
        
    newroll, newpitch, newyaw = MinidroneSensors.quaternion_to_euler_angle(mambo, newq1, newq2, newq3, newq4)
        
    roll_rate = (firstroll - newroll) * 0.5
    pitch_rate = (firstpitch - newpitch) * 0.5
    yaw_rate = (firstyaw - newyaw) * 0.5
        
    return roll_rate, pitch_rate, yaw_rate

def current_drone_state(mambo,last_x,last_y,last_z,last_time):
    current_pos = get_position(mambo,last_x,last_y,last_z,last_time)
    current_vel = get_velocity(mambo)
    current_att = get_attitude(mambo)
    current_atr = get_attitude_rate(mambo)
    
    current_state_drone = current_pos + current_vel + current_att + current_atr 
    
    return current_state_drone

def currenttimestep(mambo):
    testtime = time.time()
    
    return testtime


if __name__ == "__main__":
    #from pyparrot.Minidrone import Mambo
    #from state_estimation.state_estimator_testtwo import MamboStateOne

    # If you are using BLE: you will need to change this to the address of YOUR mambo
    # if you are using Wifi, this can be ignored
    mamboAddr = "E0:14:B1:35:3D:CB"  # "E0:14:F1:84:3D:CA"

    # make my mambo object
    # remember to set True/False for the wifi depending on if you are using the wifi or the BLE to connect

    mambo = Mambo(mamboAddr, use_wifi=True)
    #dronestates = MamboStateOne(mamboAddr, use_wifi=True)

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
            
            start_time = time.time()
            last_time = mambo.sensors.speed_ts
            
            dronestate = DroneState(x = 0, y = 0, z = mambo.sensors.altitude)
            
            #last_pos_z = 0
            
            
            while (time.time() - start_time < 8):
                #new_dronestate = current_drone_state(mambo,dronestate.z,dronestate.t_last)
                new_dronestate = current_drone_state(mambo,dronestate.x,dronestate.y,dronestate.z,last_time)
                timetest = currenttimestep(mambo)
                print("Current state: ", new_dronestate)
                print("Time: ", timetest)
                print("Speed_ts prints: ", mambo.sensors.speed_ts/1000.0)
                #dronestate = DroneState(z=new_dronestate[2],t_last = new_dronestate[13])
                dronestate = DroneState(x=new_dronestate[0],y=new_dronestate[1],z=new_dronestate[2])
                last_time = new_dronestate[4]
                print("Test drone state: ", dronestate.z)
                


            if (mambo.sensors.flying_state != "emergency"):
                dur = 1

                # test_multiple_states(mambo, dur)
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