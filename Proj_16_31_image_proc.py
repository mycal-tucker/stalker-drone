# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

"""
Demo of the ffmpeg based mambo vision code (basically flies around and saves out photos as it flies)

Author: Amy McGovern
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

    def save_pictures(self, args):
        print("in save pictures on image %d " % self.index)

        img = self.vision.get_latest_valid_picture()

        if (img is not None):
            filename = "test_image_%06d.png" % self.index
            cv2.imwrite(filename, img)
            self.index +=1
            #print(self.index)

#use to convert individual image files into a video after drone flight is done
def convert_frames_to_video(pathIn,pathOut,fps):
    frame_array = []
    files = [f for f in os.listdir(pathIn) if isfile(join(pathIn, f))]
 
    #for sorting the file names properly
    files.sort(key = lambda x: int(x[5:-4]))
 
    for i in range(len(files)):
        filename=pathIn + files[i]
        #reading each files
        img = cv2.imread(filename)
        height, width, layers = img.shape
        size = (width,height)
        print(filename)
        #inserting the frames into an image array
        frame_array.append(img)
 
    out = cv2.VideoWriter(pathOut,cv2.VideoWriter_fourcc(*'DIVX'), fps, size)
 
    for i in range(len(frame_array)):
        # writing to a image array
        out.write(frame_array[i])
    out.release()
 
    
# you will need to change this to the address of YOUR mambo
mamboAddr = "e0:14:d0:63:3d:d0"

# make my mambo object
# remember to set True/False for the wifi depending on if you are using the wifi or the BLE to connect
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
        
        pathIn= './data/'
        pathOut = 'video.avi'
        fps = mamboVision.self.fps
        convert_frames_to_video(pathIn, pathOut, fps)

    print("disconnecting")
    mambo.disconnect()
 
    
if __name__=="__main__":
    main()
