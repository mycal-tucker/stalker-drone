#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Nov 11 15:53:06 2018

@author: lenadownes
From: https://www.life2coding.com/convert-image-frames-video-file-using-opencv-python/
"""

import cv2
import os
 
from os.path import isfile, join

class im2vid:
    @staticmethod
    def convert_frames_to_video(pathIn = './images/', pathOut= 'droneVideo.avi', fps= 2):
        #pathIn is the path to the folder with saved images (default ./images)
        #pathOut is the path to the video you are creating (default droneVideo.avi)
        #fps is the frame rate of the video you are creating (default 30 frames/sec)
        frame_array = []
        files = [f for f in os.listdir(pathIn) if isfile(join(pathIn, f))]
 
        #for sorting the file names properly
        files.sort(key = lambda x: str(x[5:-4]))
 
        for i in range(len(files)):
            print(i)
            filename=pathIn + files[i]
            #reading each files
            img = cv2.imread(filename)
            height, width, layers = img.shape
            size = (width,height)
            # print(filename)
            #inserting the frames into an image array
            frame_array.append(img)

        out = cv2.VideoWriter(pathOut,cv2.VideoWriter_fourcc(*'DIVX'), fps, size)
       
        for i in range(len(frame_array)):
            # writing to a image array
            out.write(frame_array[i])

            
        out.release()
 
# im2vid.convert_frames_to_video()

#def main():
    #convert_frames_to_video(pathIn, pathOut, fps)
 
#if __name__=="__main__":
#    main()