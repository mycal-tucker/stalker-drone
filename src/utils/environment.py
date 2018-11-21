# Representation of the environment, which consists of stationary obstacles.
class Environment:
    def __init__(self, obstacles=[]):
        """
        constructor for a new environment object
        :param obstacles: a list of obstacles, where each obstacle is a list of points
        """
        self.obstacles = obstacles


    @staticmethod
    def parse_file(filename):
        """
        TODO determine how we want to save obstacles 
        """
        pass


    def add_obstacles(self, obstacles):
        """
        adds a list of obstacles to the environment object
        :param obstacles: the obstacles to be added to the environment
        """
        self.obstacles.extend(obstacles)


    def get_obstacles(self):
        """
        returns a list of obstacles in the environment
        """
        return [obstacle[:] for obstacle in self.obstacles]
