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

    def get_bound_ang(self, agent_b):
        """ Returns the left and right boundries given agent a traveling and agent b being and obstacle. """
        vec_ab = agent_b.pos - self.pos
        theta = math.atan2()

        return ang_right, ang_left


