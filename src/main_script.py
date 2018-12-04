from cinematic_waypoints.cinematic_controller import CinematicController
from cinematic_waypoints.waypoint_generator.ngon_waypoint_generator import NGonWaypointGenerator
from cinematic_waypoints.waypoint_generator.yaw_waypoint_generator import YawWaypointGenerator
from person_detection.image_saver import ImageSaver
from person_detection.tf_detector import TFDetector
from state_estimation.new_state_estimator import NewStateEstimator
from smooth_control.smooth_controller import SmoothController
from utils.im2vid import im2vid
from pyparrot.Minidrone import Mambo
from pyparrot.DroneVision import DroneVision
import numpy as np 
import cv2

# Main script for executing a flight.
# Strings together the different parts of the whole project to get a nice flight.
# General form is a big while loop that does the following:
# 1) Read an image from the drone camera and create a bounding box of the humans.
# 2) Read the current state of the drone.
# 3) Create cinematic waypoints for the drone to fly to.
# 4) Create a smooth trajectory through the waypoints.
# 5) Command the drone to follow the trajectory.

# waypoint_generator = NGonWaypointGenerator(n=4)
waypoint_generator = YawWaypointGenerator()
cinematic_controller = CinematicController(waypoint_generator=waypoint_generator)
#
mamboAddr = "e0:14:d0:63:3d:d0"  # Doesn't matter
mambo = Mambo(mamboAddr, use_wifi=True)
print("About to connect to mambo.")
mambo.connect(num_retries=3)
print("Connected to mambo.")
mambo.smart_sleep(1)
mambo.ask_for_state_update()
mambo.smart_sleep(1)

print("taking off!")
mambo.safe_takeoff(5)

# latest_image = cv2.imread('images/test_image.png') #Test image to debug, 1 bounding box

image_saver = ImageSaver(mambo)
# Update paths as needed based on working directory.
tf_detector = TFDetector(model_filepath='person_detection/ssdlite_mobilenet_v2_coco_2018_05_09/frozen_inference_graph.pb',
                         label_filepath='person_detection/mscoco_label_map.pbtxt')


state_estimator = NewStateEstimator(mambo)
smooth_controller = SmoothController(mambo,state_estimator)
firstrun = True
itercounter = 0
max_num_iterations = 3
done = False
while not done:
    # try:
    print("Starting loop, iteration number ", itercounter)
    # 1) Read an image from the drone camera and create a bounding box of the humans.
    latest_image = image_saver.get_latest_image()  # FIXME: this is not returning the latest image.

    if firstrun: #super hacky way to not init the tf session everytime 
    	bounding_boxes = tf_detector.detect_bounding_box(latest_image, visualize=True,firstrun=True)
    	firstrun = False
    else:
    	bounding_boxes = tf_detector.detect_bounding_box(latest_image, visualize=True)

    print(bounding_boxes)
    if bounding_boxes is not None:
        print("Got these many bounding boxes:", len(bounding_boxes))
    else:
        print("No bounding boxes.")

    # 2) Read the current state of the drone.
    drone_state, _ = state_estimator.get_current_drone_state()  # Ignore second argument because it's just a time.
    print("Current state is ", drone_state)

    # 3) Create cinematic waypoints for the drone to fly to.
    cinematic_controller.update_latest_bbs(bounding_boxes) 
    cinematic_controller.update_latest_drone_state(drone_state)
    cinematic_waypoints = cinematic_controller.generate_waypoints()

    # 4) Create a smooth trajectory through the waypoints and make the drone fly through it.
    smooth_controller.smooth_gen(cinematic_waypoints)
    # TODO: if this doesn't work, maybe add smart sleep stuff.

    itercounter += 1
    done = itercounter >= max_num_iterations
    # Throw in some termination conditions if we want. This could be time-based, or we just rely on
    # ctrl-C interrupts.
    # We may also want to throw in a sleep here, depending on how quickly these processes run.
    # except Exception as e:
    #     print("CAUGHT AN ERROR!:", e)
    #     done = True

# Exited the loop, so we're done and want to land
print("landing now. Flying state is %s" % mambo.sensors.flying_state)
mambo.safe_land(5)
mambo.smart_sleep(5)
print("disconnect")
mambo.disconnect()
im2vid.convert_frames_to_video()