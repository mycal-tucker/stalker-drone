import numpy as np
from smooth_control.move_commands import move


class SmoothController:
    def __init__(self, mambo):
        self.mambo = mambo

    def smooth_gen(self, drone_state, cinematic_waypoints, duration=1):
        # list of drone states
        goal = cinematic_waypoints[0]

        Dx = goal.x - drone_state.x
        Dy = goal.y - drone_state.y
        dz = goal.z - drone_state.z
        dyaw = goal.yaw - drone_state.yaw

        dx = Dx * np.sin(np.deg2rad(-drone_state.yaw))
        dy = Dy * np.cos(np.deg2rad(-drone_state.yaw))

        move(self.mambo, dx=dx, dy=dy, dz=dz, dyaw=dyaw, duration=duration)
