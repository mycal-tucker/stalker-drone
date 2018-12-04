"""
Move commands
"""

import numpy as np

def interp(val, max_value):
    val = np.abs(val)
    if val > max_value:
        val = max_value
    # TODO: min return value should be 10% or so. Related with thresh_pos
    return val/max_value*100


def move(mambo, dx, dy, dz, dyaw, duration):

    thresh_pos = 0.3
    thresh_yaw = 5
    dist_ref = 6  # m
    yaw_ref = 400

    max_yaw = 90
    if np.abs(dyaw) > max_yaw:
        dyaw = np.sign(dyaw)*max_yaw

    roll = pitch = vertical_movement = yaw = 0

    if np.abs(dx) > thresh_pos:
        roll = np.sign(dx)*interp(dx, dist_ref)

    if np.abs(dy) > thresh_pos:
        pitch = -np.sign(dy)*interp(dy, dist_ref)

    if np.abs(dz) > thresh_pos:
        vertical_movement = -np.sign(dz)*interp(dz, dist_ref)

    if np.abs(dyaw) > thresh_yaw:
        yaw = np.sign(dyaw)*interp(dyaw, yaw_ref)

    mambo.fly_direct(roll=roll, pitch=pitch, yaw=yaw, vertical_movement=vertical_movement, duration=duration)


def move_forward(mambo, percent, duration):
    print("move forward")
    mambo.fly_direct(roll=0, pitch=percent, yaw=0, vertical_movement=0, duration=duration)
    mambo.smart_sleep(duration)

def move_backward(mambo, percent, duration):
    print("move backward")
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
