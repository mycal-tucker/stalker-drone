# Representation of the drone state, using linear and angular poses and velocities.
class DroneState:
    def __init__(self, x=0, y=0, z=0, roll=0, pitch=0, yaw=0,
                 x_dot=0, y_dot=0, z_dot=0, roll_dot=0, pitch_dot=0, yaw_dot=0):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.x_dot = x_dot
        self.y_dot = y_dot
        self.z_dot = z_dot
        self.roll_dot = roll_dot
        self.pitch_dot = pitch_dot
        self.yaw_dot = yaw_dot

    def get_position(self):
        return self.x, self.y, self.z

    def get_attitude(self):
        return self.roll, self.pitch, self.yaw

    def get_linear_velocities(self):
        return self.x_dot, self.y_dot, self.z_dot

    def get_angular_velocities(self):
        return self.roll_dot, self.pitch_dot, self.yaw_dot
