import unittest
from pyparrot.Minidrone import Mambo
import time

from state_estimation.new_state_estimator import NewStateEstimator
from utils.drone_state import DroneState


# Class for testing the state estimator and running sample scripts
class TestStateEstimator(unittest.TestCase):

    # TODO: It would be nice to mirror setUp with a landing and disconnect at the end.
    def setUp(self):
        # If you are using BLE: you will need to change this to the address of YOUR mambo
        # if you are using Wifi, this can be ignored
        mamboAddr = "E0:14:B1:35:3D:CB"  # "E0:14:F1:84:3D:CA"

        # make my mambo object
        # remember to set True/False for the wifi depending on if you are using the wifi or the BLE to connect

        mambo = Mambo(mamboAddr, use_wifi=True)
        # dronestates = MamboStateOne(mamboAddr, use_wifi=True)

        print("trying to connect")
        success = mambo.connect(num_retries=3)
        print("connected: %s" % success)

        self.mambo = mambo

        self.state_estimator = NewStateEstimator(mambo)

    def test_state_estimator(self):
        # get the state information
        print("sleeping")
        self.mambo.smart_sleep(2)  # sleeps the requested number of seconds but wakes up for notifications
        self.mambo.ask_for_state_update()
        self.mambo.smart_sleep(2)

        print("taking off!")
        self.mambo.safe_takeoff(5)

        start_time = time.time()
        last_time = self.mambo.sensors.speed_ts

        test_duration = 8  # in seconds

        while time.time() - start_time < test_duration:
            estimated_drone_state, time_of_estimate = self.state_estimator.get_current_drone_state()

            #new_dronestate = current_drone_state(mambo, dronestate.x, dronestate.y, dronestate.z, last_time)
            current_time = self.state_estimator.currenttimestep()
            print("Current state: ", estimated_drone_state)
            print("Time: ", current_time)
            print("Speed_ts prints: ", self.mambo.sensors.speed_ts / 1000.0)
            print("Test drone state: ", estimated_drone_state.z)

        print("landing now. Flying state is %s" % self.mambo.sensors.flying_state)
        self.mambo.safe_land(5)
        self.mambo.smart_sleep(5)
        print("disconnect")
        self.mambo.disconnect()


if __name__ == '__main__':
    unittest.main()
