"""
Move commands
"""

import numpy as np

def move_forward(mambo, percent, duration):
    print("move forward")
    #print("flying state is %s" % mambo.sensors.flying_state)
    mambo.fly_direct(roll=0, pitch=percent, yaw=0, vertical_movement=0, duration=duration)
    mambo.smart_sleep(duration)

def move_back(mambo, percent, duration):
    print("move back")
    mambo.fly_direct(roll=0, pitch=-percent, yaw=0, vertical_movement=0, duration=duration)
    mambo.smart_sleep(duration)

def move_left(mambo, percent, duration):
    print("move left")
    mambo.fly_direct(roll=-percent, pitch=0, yaw=0, vertical_movement=0, duration=duration)
    mambo.smart_sleep(duration)

def move_right(mambo, percent, duration):
    print("move right")
    mambo.fly_direct(roll=percent, pitch=0, yaw=0, vertical_movement=0, duration=duration)
    mambo.smart_sleep(duration)

def move_up(mambo, percent, duration):
    print("move up")
    mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=percent, duration=duration)
    mambo.smart_sleep(duration)

def move_down(mambo, percent, duration):
    print("move down")
    mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=-percent, duration=duration)
    mambo.smart_sleep(duration)

def yaw(mambo, degrees):
    # Turns the minidrone in place the specified number of degrees from -180 to 180 (clockwise is positive)
    print("yaw")
    mambo.turn_degrees(degrees)  # more accurate than fly_direct
    mambo.smart_sleep(duration)
