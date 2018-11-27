"""
Move commands
"""

import numpy as np

def move_forward(mambo, percent, duration):
    print("move forward")
    print("flying state is %s" % mambo.sensors.flying_state)
    mambo.fly_direct(roll=0, pitch=percent, yaw=0, vertical_movement=0, duration=duration)

def move_back(mambo, percent, duration):
    print("move back")
    print("flying state is %s" % mambo.sensors.flying_state)
    mambo.fly_direct(roll=0, pitch=-percent, yaw=0, vertical_movement=0, duration=duration)

def move_left(mambo, percent, duration):
    print("move left")
    print("flying state is %s" % mambo.sensors.flying_state)
    mambo.fly_direct(roll=-percent, pitch=0, yaw=0, vertical_movement=0, duration=duration)

def move_right(mambo, percent, duration):
    print("move right")
    print("flying state is %s" % mambo.sensors.flying_state)
    mambo.fly_direct(roll=percent, pitch=0, yaw=0, vertical_movement=0, duration=duration)

def move_up(mambo, percent, duration):
    print("move up")
    print("flying state is %s" % mambo.sensors.flying_state)
    mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=-percent, duration=duration)

def move_down(mambo, percent, duration):
    print("move down")
    print("flying state is %s" % mambo.sensors.flying_state)
    mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=percent, duration=duration)

def yaw(mambo, degrees):
    # Turns the minidrone in place the specified number of degrees. The range is -180 to 180
    mambo.turn_degrees(degrees)  # more accurate than fly_direct
