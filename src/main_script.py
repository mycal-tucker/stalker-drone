from cinematic_waypoints.cinematic_controller import CinematicController
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
while not done:
    # 1) Read an image from the drone camera and create a bounding box of the humans.
    bounding_boxes = None  # TODO call appropriate methods
    # 2) Read the current state of the drone.
    drone_state = None  # TODO call appropriate methods
    # 3) Create cinematic waypoints for the drone to fly to.
    cinematic_controller.update_latest_bbs(bounding_boxes)
    cinematic_controller.update_latest_drone_state(drone_state)
    cinematic_waypoints = cinematic_controller.generate_waypoints()
    cinematic_controller.update_cinematic_waypoints(cinematic_waypoints)


    # 4) Create a smooth trajectory through the waypoints.
    # TODO
    # 5) Command the drone to follow the trajectory.
    # TODO

    # Throw in some termination conditions if we want. This could be time-based, or we just rely on
    # ctrl-C interrupts.
    # We may also want to throw in a sleep here, depending on how quickly these processes run.
    pass
