import numpy as np
from smooth_control.move_commands import move


class SmoothController:

    def __init__(self, mambo, state_estim):
        self.mambo = mambo
        self.state_estim = state_estim
        self.thresh_dist = 0.2
        self.thresh_yaw = 5


    def smooth_gen(self, cinematic_waypoints, duration=1):
        # list of drone states

        close = False
        while not close:
            drone_state, _ = self.state_estim.get_current_drone_state()
            goal = cinematic_waypoints[0]

            Dx = goal.x - drone_state.x
            Dy = goal.y - drone_state.y
            dz = goal.z - drone_state.z
            dyaw = goal.yaw - drone_state.yaw

            dx = Dx * np.sin(np.deg2rad(-drone_state.yaw))
            dy = Dy * np.cos(np.deg2rad(-drone_state.yaw))

            move(self.mambo, dx=dx, dy=dy, dz=dz, dyaw=dyaw, duration=duration)

            distance = np.sqrt(dx**2 + dy**2 + dz**2)
            if distance < self.thresh_dist and dyaw < self.thresh_yaw:
                close = True

