#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov 29 16:38:53 2018

@author: lenadownes
"""

from pyparrot.Minidrone import Mambo
from pyparrot.DroneVision import DroneVision
import threading
import cv2
import time
import numpy as np
import os

from os.path import isfile, join


class UserVision:
    def __init__(self, vision):
        self.index = 0
        self.vision = vision
mambo = Mambo(mamboAddr, use_wifi=True)
print("trying to connect to mambo now")
success = mambo.connect(num_retries=3)
print("connected: %s" % success)

if (success):
    # get the state information
    print("sleeping")
    mambo.smart_sleep(1)
    mambo.ask_for_state_update()
    mambo.smart_sleep(1)

    print("Preparing to open vision")
    mamboVision = DroneVision(mambo, is_bebop=False, buffer_size=30)
    userVision = UserVision(mamboVision)
    mamboVision.set_user_callback_function(userVision.save_pictures, user_callback_args=None)
    success = mamboVision.open_video() #Open the video stream using ffmpeg for capturing and processing
    print("Success in opening vision is %s" % success)

    if (success):
        print("Vision successfully started!")

        img = mamboVision.get_latest_valid_picture()
        
        #here should be input to detect.py
        
        # done doing vision demo
        print("Ending the sleep and vision")
        mamboVision.close_video()

        mambo.smart_sleep(5)


    print("disconnecting")
    mambo.disconnect()
 
    
if __name__=="__main__":
    main()