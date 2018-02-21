import numpy as np


class Agent(object):

    def __init__(self, start_pos, goal_pos, start_vel = np.zeros(2)):
        self.pos = start_pos
        self.goal = goal_pos
        self.vel = start_vel

    def distance_to(self, point):
        """Returns the euclidian distance from the agent to a point"""
        return np.linalg.norm(self.pos - point)


    def get_avoidance_vels(self, agent):
        """ Returns the avoidance velocities"""
        vels = []

        return vels


