import numpy as np
import matplotlib
from matplotlib.lines import Line2D
from matplotlib.patches import Arrow, Polygon, Circle
from matplotlib.collections import PatchCollection
import matplotlib.pyplot as plt


class WaypointVisualization():
    def __init__(self, drone_states, person_state=None, environment=None):
        """
        constructor for a waypoint visualization object
        :param drone_states: list of drone state objects that represent the drone trajectory
        :param person_state: optional, a person state object representing the person
        :param environment: optional, an environment object that desribes the obstacles in the environment
        :returns: a new waypoint visualization object
        :requires: that the drone states list contains at least one drone state object
        """
        # user supplied parameters for plotting
        self.drone_states  = drone_states
        self.person_state  = person_state
        self.environment   = environment
        assert(len(drone_states) > 0), 'Warning! need at least one waypoint to vislualize'

        # plotting-specific parameters
        self.color_obstacle_fill    = 'skyblue'
        self.color_obstacle_edge    = 'navy'
        self.color_person_fill      = 'salmon'
        self.color_person_edge      = 'red'
        self.color_drone_path       = 'peachpuff'
        self.color_drone_arrow      = 'darkorange'
        self.color_drone_first      = 'darkgreen'
        self.color_drone_final      = 'firebrick'
        self.color_drone_risk       = 'lightcoral'
        self.size_edge              = 3
        self.size_buffer            = 2
        self.width_drone            = 0.8
        self.length_drone           = 1
        self.opacity                = 1
        self.figsize                = (6.5,6.5)
        self.title_font             = {'fontname':'Arial',
                                       'size':'16',     
                                       'color':'black', 
                                       'weight':'bold'}


    def set_drone_states(self, drone_states):
        """
        updates the list of drone states to be visualized
        :param drone_states: the list of states to be visualized
        """
        self.drone_states = drone_states


    def set_person_state(self, person_state):
        """
        updates the person state to be visualized
        :param person_state: the person to be visualized, assumed to be stationary
        """
        self.person_state = person_state


    def set_environment(self, environment):
        """
        updates the environment to be visualized
        :param environment: the environment to be visualized, obstacles assumed to be stationary
        """
        self.environment = environment


    def plot(self, filename=None):
        """
        plots the drone trajectory along with environment obstacles and person obstacles
        :param filename: name of the file to be saved after plot is generated
            + if filename is not specified, plot is displayed to user rather than saved to file
        """
        # get the limits of the drone trajectory 
        x_min = min([waypoint.get_position()[0] for waypoint in self.drone_states])
        x_max = max([waypoint.get_position()[0] for waypoint in self.drone_states])
        y_min = min([waypoint.get_position()[1] for waypoint in self.drone_states])
        y_max = max([waypoint.get_position()[1] for waypoint in self.drone_states])

        # depth order for plotting
        z_obstacle    = 2
        z_person      = 3
        z_drone_path  = 4
        z_drone_arrow = 5

        # initialize the figure
        fig = plt.figure(figsize=self.figsize)
        ax  = fig.add_subplot(111)

        # plot all waypoints in the drone state 
        for i in range(len(self.drone_states)-1):
            current_x = self.drone_states[i].get_position()[0]
            current_y = self.drone_states[i].get_position()[1]
            current_t = self.drone_states[i].get_attitude()[2]
            next_x    = self.drone_states[i+1].get_position()[0]
            next_y    = self.drone_states[i+1].get_position()[1]
            distance  = ((next_y - current_y)**2 + (next_x - current_x)**2)**0.5
            angle     = np.arctan2(next_y - current_y, next_x - current_x) * (180/np.pi)

            # plot the linearly interpolated path between the drone waypoints 
            ax.add_line(Line2D(xdata        = (current_x, next_x),
                               ydata        = (current_y, next_y),
                               linewidth    = self.width_drone*10,
                               color        = self.color_drone_path,
                               zorder       = z_drone_path))
 
            # plot the arrow representing the drones orientation at each waypoint
            ax.add_patch(Arrow(x            = current_x - (self.length_drone/2.0)*np.cos(current_t),
                               y            = current_y - (self.length_drone/2.0)*np.sin(current_t),
                               dx           = (self.length_drone)*np.cos(current_t),
                               dy           = (self.length_drone)*np.sin(current_t),
                               facecolor    = self.color_drone_arrow,
                               width        = self.width_drone,
                               zorder       = z_drone_arrow))

        # plot the last drone waypoint with different color scheme
        final_x = self.drone_states[-1].get_position()[0]
        final_y = self.drone_states[-1].get_position()[1]
        final_t = self.drone_states[-1].get_attitude()[2]
        ax.add_patch(Arrow(x                = final_x - (self.length_drone/2.0)*np.cos(final_t),
                           y                = final_y - (self.length_drone/2.0)*np.sin(final_t),
                           dx               = (self.length_drone)*np.cos(final_t),
                           dy               = (self.length_drone)*np.sin(final_t),
                           facecolor        = self.color_drone_final,
                           width            = self.width_drone,
                           zorder           = z_drone_arrow))

        # plot the last drone waypoint with different color scheme
        first_x = self.drone_states[0].get_position()[0]
        first_y = self.drone_states[0].get_position()[1]
        first_t = self.drone_states[0].get_attitude()[2]
        ax.add_patch(Arrow(x                = first_x - (self.length_drone/2.0)*np.cos(first_t),
                           y                = first_y - (self.length_drone/2.0)*np.sin(first_t),
                           dx               = (self.length_drone)*np.cos(first_t),
                           dy               = (self.length_drone)*np.sin(first_t),
                           facecolor        = self.color_drone_first,
                           width            = self.width_drone,
                           zorder           = z_drone_arrow))

        # plot the person, assumed to be stationary
        if self.person_state != None:
            person_x, person_y, person_r    = self.person_state.get_person_state()
            ax.add_patch(Circle(xy          = (person_x, person_y),
                                radius      = person_r,
                                facecolor   = self.color_person_fill,
                                edgecolor   = self.color_person_edge,
                                lw          = self.size_edge,
                                zorder      = z_person))

        if self.environment != None:
            for obstacle in self.environment.get_obstacles():
                ax.add_patch(Polygon(xy     = np.array(obstacle), 
                                    fill    = True, 
                                    fc      = self.color_obstacle_fill, 
                                    ec      = self.color_obstacle_edge, 
                                    lw      = self.size_edge,
                                    zorder  = z_obstacle))

        # set title and other plot styles, then generate plot 
        plt.title('Drone Stalker Trajectory', self.title_font)
        ax.grid()
        ax.axis('equal')
        ax.set_xlim(x_min - self.size_buffer, x_max + self.size_buffer)
        ax.set_ylim(y_min - self.size_buffer, y_max + self.size_buffer)
        if filename == None:    plt.show()
        else:                   plt.savefig(filename)
