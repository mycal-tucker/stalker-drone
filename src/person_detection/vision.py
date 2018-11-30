"""
Demo of the ffmpeg based mambo vision code (basically flies around and saves out photos as it flies)

Author: Amy McGovern
"""
from __future__ import print_function
from imutils.object_detection import non_max_suppression
from imutils import paths
from imutils import contours
from pyparrot.Minidrone import Mambo
from pyparrot.DroneVision import DroneVision

import threading
import cv2
import time
import numpy as np
import argparse
import imutils
import os


hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

# set this to true if you want to fly for the demo
testFlying = False

class UserVision:
    def __init__(self, vision):
        self.index = 0
        self.vision = vision

    def save_pictures(self, args):
        print("in save pictures on image %d " % self.index)

        image = self.vision.get_latest_valid_picture()



        print("detecting person with HOG")

        image = self.vision.get_latest_valid_picture()
        image = imutils.resize(image, width=min(400, image.shape[1]))
        orig = image.copy()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) #Converts image to grayscale

        #Detects Person
        (rects, weights) = hog.detectMultiScale(gray, winStride=(4, 4), padding=(8, 8), scale=1.05)

        # draw the bounding boxes
        for (x, y, w, h) in rects:
            cv2.rectangle(gray, (x, y), (x + w, y + h), (0, 0, 255), 2)

        # apply non-maxima suppression to the bounding boxes using a
        # fairly large overlap threshold to try to maintain overlapping
        # boxes that are still people
        rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
        # print(rects)
        pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)

        centX = 0
        # draw the final bounding boxes
        for (xA, yA, xB, yB) in pick:
            centX = abs(xA+xB)//2 #calculates centroid of final bounding boxes
            centY = abs(yA+yB)//2

            width = abs(xB-xA) #Calculates height and width of final bounding boxes
            height = abs(yB-yA)

            cv2.rectangle(gray, (xA, yA), (xB, yB), (0, 255, 0), 2)

            # cv2.rectangle(gray, (width, height), (width+10, height+10), (0, 255, 0), 2)
            # cv2.rectangle(gray, (centX, centY), (centX+10, centY-10), (0, 255, 0), 2) #Draws rectangle near centroid
             
        # cv2.imwrite("Test_Detected" + str(self.index) + ".png",gray) #saves image to working directory

        if (image is not None):
            filename = "test_image_%06d.png" % self.index
            cv2.imwrite(filename, gray)
            self.index +=1
            #print(self.index)

    def person_detection(self,args):
        print("detecting person with HOG")

        image = self.vision.get_latest_valid_picture()
        image = imutils.resize(image, width=min(400, image.shape[1]))
        orig = image.copy()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) #Converts image to grayscale

        #Detects Person
        (rects, weights) = hog.detectMultiScale(gray, winStride=(4, 4), padding=(8, 8), scale=1.05)

        # draw the original bounding boxes
        for (x, y, w, h) in rects:
            cv2.rectangle(gray, (x, y), (x + w, y + h), (0, 0, 255), 2)

        # apply non-maxima suppression to the bounding boxes using a
        # fairly large overlap threshold to try to maintain overlapping
        # boxes that are still people
        rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
        # print(rects)
        pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)

        centX = 0
        # draw the final bounding boxes
        for (xA, yA, xB, yB) in pick:
            centX = abs(xA+xB)//2 #calculates centroid of final bounding boxes
            centY = abs(yA+yB)//2

            width = abs(xB-xA) #Calculates height and width of final bounding boxes
            height = abs(yB-yA)

            cv2.rectangle(gray, (xA, yA), (xB, yB), (0, 255, 0), 2)

            # cv2.rectangle(gray, (width, height), (width+10, height+10), (0, 255, 0), 2)
            # cv2.rectangle(gray, (centX, centY), (centX+10, centY-10), (0, 255, 0), 2) #Draws rectangle near centroid
             

        if (image is not None):
            filename = "test_image_%06d.png" % self.index
            cv2.imwrite(filename, img)
            self.index +=1
            #print(self.index)






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
    # mamboVision.set_user_callback_function(userVision.person_detection, user_callback_args=None)
    # mamboVision.set_user_callback_function(userVision.display_latest_frame,user_callback_args=None)
    success = mamboVision.open_video()
    print("Success in opening vision is %s" % success)

    if (success):
        print("Vision successfully started!")
        #removed the user call to this function (it now happens in open_video())
        #mamboVision.start_video_buffering()

        #Calls to TensorFlow Person Detection

        while (success):
            print("on")
            # person_detection()

            

        # done doing vision demo
        print("Ending the sleep and vision")
        mamboVision.close_video()

        mambo.smart_sleep(5)

    print("disconnecting")
    mambo.disconnect()
