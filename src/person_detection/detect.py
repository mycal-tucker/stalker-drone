#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov  8 10:03:52 2018

@author: mayanasr

Source of the code: https://www.pyimagesearch.com/2015/11/09/pedestrian-detection-opencv/

"""

# import the necessary packages
from __future__ import print_function
from imutils.object_detection import non_max_suppression
from imutils import paths
from imutils import contours
import numpy as np
import argparse
import imutils
import cv2
import os

#files = os.listdir() 
#files = [f if f.split(".")[-1] == "jpg" for f in files]
#print(files)   

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--images", required=True, help="path to images directory")
args = vars(ap.parse_args())
 
# initialize the HOG descriptor/person detector
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

# loop over the image paths
imgnum = 0
for imagePath in paths.list_images(args["images"]):
    # load the image and resize it to (1) reduce detection time
    # and (2) improve detection accuracy
    image = cv2.imread(imagePath)
    image = imutils.resize(image, width=min(400, image.shape[1]))
    orig = image.copy()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) #Converts image to grayscale
    # print("Image Shape:  " + str(gray.shape)) 


    # detect people in the image
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
    print(pick) 

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


     
    # show some information on the number of bounding boxes
    filename = imagePath[imagePath.rfind("/") + 1:]
    print("[INFO] {}: {} original boxes, {} after suppression".format(filename, len(rects), len(pick)))
     
    # show the output images
    # cv2.imshow("Gray image",gray)
    # cv2.waitKey(0)

    #write the output images
    cv2.imwrite("Test_Detected" + str(imgnum) + ".png",gray) #saves image to working directory
    imgnum += 1



    #other info to return about images

    # if centX != 0: #only prints if a person was detected
    #     print("centroidX" + str(centX))
    #     print("centroidY" + str(centY))
    #     print("width" + str(width))
    #     print("height" + str(height))


    #Super hacky code to find only images with bounding boxes
    # if centX != 0:  
    # cv2.imwrite("BoundingBoxTests" + str(imgnum) + ".png",gray) #saves image to working directory
    # imgnum += 1
