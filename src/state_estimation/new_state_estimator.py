from pyparrot.Minidrone import MinidroneSensors
import time
from utils.drone_state import DroneState


class NewStateEstimator:
    def __init__(self, mambo):
        self.mambo = mambo
        self.previous_drone_state = DroneState(x=0, y=0, z=mambo.sensors.altitude)
        self.previous_fetch_time = mambo.sensors.speed_ts

    def get_velocity(self):
        vel_x = self.mambo.sensors.speed_x
        vel_y = self.mambo.sensors.speed_y
        vel_z = self.mambo.sensors.speed_z

        return vel_x, vel_y, vel_z

    # Note: this doesn't just return x, y, z.
    # It returns x, y, z, z_compare, and the current time.
    # z_compare seems to be more accurate than z.
    def get_position(self, last_x, last_y, last_z, last_time):
        old_pos_z = last_z
        old_pos_x = last_x
        old_pos_y = last_y
        old_time = last_time
        # current_time = time.time()
        current_time = self.mambo.sensors.speed_ts
        elapsed_time = (current_time - old_time) / 1000.0
        pos_x = old_pos_x + (self.mambo.sensors.speed_x * elapsed_time)
        pos_y = old_pos_y + (self.mambo.sensors.speed_y * elapsed_time)
        pos_z = old_pos_z + (self.mambo.sensors.speed_z * elapsed_time)

        pos_z_compare = self.mambo.sensors.altitude

        return pos_x, pos_y, pos_z, pos_z_compare, current_time

    def get_attitude(self):
        q1 = self.mambo.sensors.quaternion_w
        q2 = self.mambo.sensors.quaternion_x
        q3 = self.mambo.sensors.quaternion_y
        q4 = self.mambo.sensors.quaternion_z

        roll, pitch, yaw = MinidroneSensors.quaternion_to_euler_angle(self.mambo, q1, q2, q3, q4)

        return roll, pitch, yaw

    def get_attitude_rate(self):
        firstq1 = self.mambo.sensors.quaternion_w
        firstq2 = self.mambo.sensors.quaternion_x
        firstq3 = self.mambo.sensors.quaternion_y
        firstq4 = self.mambo.sensors.quaternion_z

        firstroll, firstpitch, firstyaw = MinidroneSensors.quaternion_to_euler_angle(self.mambo, firstq1, firstq2, firstq3,
                                                                                     firstq4)

        self.mambo.ask_for_state_update()

        newq1 = self.mambo.sensors.quaternion_w
        newq2 = self.mambo.sensors.quaternion_x
        newq3 = self.mambo.sensors.quaternion_y
        newq4 = self.mambo.sensors.quaternion_z

        newroll, newpitch, newyaw = MinidroneSensors.quaternion_to_euler_angle(self.mambo, newq1, newq2, newq3, newq4)

        roll_rate = (firstroll - newroll) * 0.5
        pitch_rate = (firstpitch - newpitch) * 0.5
        yaw_rate = (firstyaw - newyaw) * 0.5

        return roll_rate, pitch_rate, yaw_rate

    # Returns current drone state and the time.
    def get_current_drone_state(self):
        # NOTE: we intentionally skip the third value because it's just noisy z.
        current_x, current_y, _, current_z, current_time = self.get_position(self.previous_drone_state.x,
                                                                             self.previous_drone_state.y,
                                                                             self.previous_drone_state.z,
                                                                             self.previous_fetch_time)
        current_vel = self.get_velocity()
        current_att = self.get_attitude()
        current_atr = self.get_attitude_rate()

        current_drone_state = DroneState(current_x, current_y, current_z,
                                         current_att[0], current_att[1], current_att[2],
                                         current_vel[0], current_vel[1], current_vel[2],
                                         current_atr[0], current_atr[1], current_atr[2])

        self.previous_drone_state = current_drone_state
        self.previous_fetch_time = current_time

        return current_drone_state, current_time

    def has_taken_off(self):
       return DroneState(x=0,y=0,z=0,roll=0,pitch=0,yaw=0)
       #return ret


    @staticmethod
    def currenttimestep():
        testtime = time.time()

        return testtime
