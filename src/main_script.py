from cinematic_waypoints.cinematic_controller import CinematicController
from person_detection.image_saver import ImageSaver
from person_detection.tf_detector import TFDetector
from state_estimation.new_state_estimator import NewStateEstimator
from pyparrot.Minidrone import Mambo
from pyparrot.DroneVision import DroneVision

# Main script for executing a flight.
# Strings together the different parts of the whole project to get a nice flight.
# General form is a big while loop that does the following:
# 1) Read an image from the drone camera and create a bounding box of the humans.
# 2) Read the current state of the drone.
# 3) Create cinematic waypoints for the drone to fly to.
# 4) Create a smooth trajectory through the waypoints.
# 5) Command the drone to follow the trajectory.

done = False
cinematic_controller = CinematicController()

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

image_saver = ImageSaver(mambo)
# Update paths as needed based on working directory.
tf_detector = TFDetector(model_filepath='person_detection/ssdlite_mobilenet_v2_coco_2018_05_09/frozen_inference_graph.pb',
                         label_filepath='person_detection/mscoco_label_map.pbtxt')

state_estimator = NewStateEstimator(mambo)
itercounter = 0
while not done:
    print("Starting loop, iteration number ", itercounter)
    # 1) Read an image from the drone camera and create a bounding box of the humans.
    latest_image = image_saver.get_latest_image()  # FIXME: this is not returning the latest image.
    bounding_boxes = tf_detector.detect_bounding_box(latest_image, visualize=True)

    # 2) Read the current state of the drone.
    drone_state = state_estimator.get_current_drone_state()
    print("Current state is ", drone_state)
    # 3) Create cinematic waypoints for the drone to fly to.
    cinematic_controller.update_latest_bbs(bounding_boxes)
    cinematic_controller.update_latest_drone_state(drone_state)
    cinematic_waypoints = cinematic_controller.generate_waypoints()

    # 4) Create a smooth trajectory through the waypoints.
    # TODO
    # 5) Command the drone to follow the trajectory.
    # TODO

    itercounter += 1
    # Throw in some termination conditions if we want. This could be time-based, or we just rely on
    # ctrl-C interrupts.
    # We may also want to throw in a sleep here, depending on how quickly these processes run.
    pass
