import tensorflow as tf
import numpy as np
import cv2
import datetime

from pyparrot.Minidrone import Mambo
from pyparrot.DroneVision import DroneVision
#import threading
#import time
#import os

#from os.path import isfile, join

# You will need to download:
#  1) a pre-trained tensorflow model which performs object detection on MSCOCO objects
#  2) the associated 90-class text labels from the MSCOCO object dataset
#
# For instance, to download the following model:
#
#     ssdlite_mobilenet_v2_coco_2018_05_09
# 
# use wget to fetch the model:
#
#   $ wget http://download.tensorflow.org/models/object_detection/ssdlite_mobilenet_v2_coco_2018_05_09.tar.gz
#   $ untar -xzvf ssdlite_mobilenet_v2_coco_2018_05_09.tar.gz
#
# and the label file:
#
#   $ wget https://raw.githubusercontent.com/tensorflow/models/master/research/object_detection/data/mscoco_label_map.pbtxt
#
#  NOTE: *******************************************************************************
#                                                                                      *
#   DO NOT INSTALL TENSORFLOW ON YOUR TEAM DRONE.                                      *
#   You may __only__ install tensorflow on your TEAM LAPTOP or your personal laptop.   *
#   MAKE SURE you are NOT ssh'd into the drone when installing tensorflow!             *
#                                                                                      *
#  *************************************************************************************
# 
# We strongly recommend this installation procedure on the team laptop, 
# as it is the only option which has been tested to work alongside ROS:
#   $ pip install --user tensorflow
#
# Note that other models with different speed/accuracy tradeoffs are available,
# but ssdlite_mobilenet_v2_coco_2018_05_09 is recommended for the challenge.
# If interested, consult the Detection Model Zoo for other pre-trained models:
# https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md
    




 #### NOTES 
'''
"Errno Port something in use"
the problem is that the python scripts leave the UDP ports on if you cancel it before it mambo.disconnects
lsof -i     to check which port is still on 
find the PID of the python script
kill -9 [pid number] to kill

restart!

"Trouble connecting to camera"
something about powercycling the drone, mark said it once
just take battery out and put it back in




'''

class UserVision:
    def __init__(self, vision):
        self.index = 0
        self.vision = vision

    def save_pictures(self, args):
        print("in save pictures on image %d " % self.index)

        img = self.vision.get_latest_valid_picture()

        if img is not None:
            filename = "test_image_%06d.png" % self.index
            cv2.imwrite(filename, img)
            self.index += 1
            # Save another copy of the same image to a hardcoded filename, which
            # we'll use as the latest image
            magic_filename = "latest_image.png"
            cv2.imwrite(magic_filename, img)
            #print(self.index)
            
MODEL         = 'ssdlite_mobilenet_v2_coco_2018_05_09/frozen_inference_graph.pb'
LABELS        = 'mscoco_label_map.pbtxt'
REPORT_TIMING = False

def read_labels(labelfile):
    with open(labelfile, 'r') as fid:
        lines = fid.readlines()
    ids          = lines[2::5]
    displaynames = lines[3::5]
    ids = [int(ix.strip("id: ").rstrip()) for ix in ids]
    displaynames = [d.lstrip("display_name: ")[1:].rstrip("\"\n") for d in displaynames]
    labelmap = {}
    for ix,dn in zip(ids,displaynames):
        labelmap[ix] = dn
    return labelmap

def run_inference_for_single_image(image, sess, tensor_dict, image_tensor):
            if REPORT_TIMING:
                tstart = datetime.datetime.now()
            output_dict = sess.run(tensor_dict,
                             feed_dict={image_tensor: np.expand_dims(image, 0)})
            if REPORT_TIMING:
                tdone = datetime.datetime.now()
                print("Inference time: {}".format(tdone - tstart))

            # all outputs are float32 numpy arrays, so convert types as appropriate
            output_dict['num_detections'] = int(output_dict['num_detections'][0])
            output_dict['detection_classes'] = output_dict[
                'detection_classes'][0].astype(np.uint8)
            output_dict['detection_boxes'] = output_dict['detection_boxes'][0]
            output_dict['detection_scores'] = output_dict['detection_scores'][0]
            return output_dict

def cv2_visualize_results(image, output_dict, labels, detect_thresh=0.5):
    imheight, imwidth, _ = image.shape
    for ix,score in enumerate(output_dict['detection_scores']):  
        if score > detect_thresh:
            [ymin,xmin,ymax,xmax] = output_dict['detection_boxes'][ix]
            classid   = output_dict['detection_classes'][ix]
            classname = labels[classid]
            
            if classid == 1: #refers to person
                
                x,y = (int(xmin * imwidth), int(ymin * imheight))
                X   = int(xmax * imwidth)
                Y   = int(ymax * imheight)
                
                # bounding box viz
                cv2.rectangle(image, (x,y), (X,Y), (0,255,0), 2)
                
                # class label viz
                label_background_color = (0, 255, 0)
                
                label_text = "{}".format(classname)
                label_text_color = (0,0,0)
    
                label_size = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 1, 1)[0]
                label_left = x
                label_top = y - label_size[1]
                if (label_top < 1):
                    label_top = 1
                label_right = label_left + label_size[0]
                label_bottom = label_top + label_size[1]
                cv2.rectangle(image, (label_left - 1, label_top - 1), (label_right + 1, label_bottom + 1),
                              label_background_color, -1)
    
                # label text above the box
                cv2.putText(image, label_text, (label_left, label_bottom), cv2.FONT_HERSHEY_SIMPLEX, 1, label_text_color, 2,cv2.LINE_AA)


if __name__ == "__main__":
    # read the model graph and labels from disk:
    detection_graph = tf.Graph()
    with detection_graph.as_default():
        od_graph_def = tf.GraphDef()
        with tf.gfile.GFile(MODEL, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')

    labels = read_labels(LABELS)

    # begin a tensorflow session
    detection_graph.as_default()
    sess = tf.Session(graph=detection_graph)
    # Get handles to input and output tensors
    ops = detection_graph.get_operations()
    all_tensor_names = {output.name for op in ops for output in op.outputs}
    tensor_dict = {}
    for key in ['num_detections', 'detection_boxes', 'detection_scores', 'detection_classes']:
        tensor_name = key + ':0'
        if tensor_name in all_tensor_names:
            tensor_dict[key] = detection_graph.get_tensor_by_name(
                      tensor_name)
    image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')

    # begin processing the default camera feed for your device:


    mamboAddr = "e0:14:d0:63:3d:d0"
    mambo = Mambo(mamboAddr, use_wifi=True)
    print("trying to connect to mambo now")
    success = mambo.connect(num_retries=3)
    print("connected: %s" % success)

    while (success):
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

        print("Vision successfully started!")

        frame = mamboVision.get_latest_valid_picture()
    
        #here should be input to detect.py
    
        # run inference
        res = run_inference_for_single_image(frame, sess, tensor_dict, image_tensor)
        print("frame")
        # view the bounding boxes:
        cv2_visualize_results(frame, res, labels)
        cv2.imshow('frame', frame)

        # done doing vision demo
        print("Ending the sleep and vision")
        mamboVision.close_video()

        mambo.smart_sleep(5)


        print("disconnecting")
        mambo.disconnect()
 
    
#if __name__=="__main__":
#    main()
            
#    cap = cv2.VideoCapture(0)
#    while True:
#        ret, frame = cap.read()
#
#        # run inference
#        res = run_inference_for_single_image(frame, sess, tensor_dict, image_tensor)
#
#        # view the bounding boxes:
#        cv2_visualize_results(frame, res, labels)
#        cv2.imshow('frame', frame)
#
#        # quit if the user presses 'q' on the keyboard:
#        if cv2.waitKey(1) & 0xFF == ord('q'):
#            break
#    cap.release()
#    cv2.destroyAllWindows()
#



## this example is partially based on code from tensorflow.org and Intel's ncappzoo