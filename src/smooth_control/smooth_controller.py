import numpy as np
from smooth_control.move_commands import move


class SmoothController:

    def __init__(self, mambo, state_estim):
        self.mambo = mambo
        self.state_estim = state_estim
        self.thresh_dist = 0.5
        self.thresh_yaw = 20


    def smooth_gen(self, cinematic_waypoints, duration=1):
        # list of drone states

        close = False
        it = 0
        while not close:
            print()
            it += 1
            print("it:", it)

            drone_state, _ = self.state_estim.get_current_drone_state()
            goal = cinematic_waypoints[0]
            print("drone_state:", drone_state)
            print("goal_state:", goal)

            # pyparrot's +x is our -y (forward)
            # pyparrot's +y is our +x (right)
            # pyparrot's +z is our -z (up)

            Dx = goal.x - drone_state.y
            Dy = goal.y - (-drone_state.x)
            dz = goal.z - (-drone_state.z)
            dyaw = goal.yaw - drone_state.yaw

            # conversion to relative coordinates
            dx = Dx * np.sin(np.deg2rad(-drone_state.yaw))
            dy = Dy * np.cos(np.deg2rad(-drone_state.yaw))

            print("dx", dx)
            print("dy", dy)
            print("dz", dz)
            print("dyaw", dyaw)

            move(self.mambo, dx=dx, dy=dy, dz=dz, dyaw=dyaw, duration=duration)

            distance = np.sqrt(dx**2 + dy**2 + dz**2)
            if distance < self.thresh_dist and dyaw < self.thresh_yaw:
                close = True

        print("Done! Close enough")

