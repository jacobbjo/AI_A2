import numpy as np
from math import *


class Agent(object):

    def __init__(self, start_pos, goal_pos, radius, start_vel = np.zeros(2)):
        self.pos = start_pos
        self.goal = goal_pos
        self.vel = start_vel
        self.r = radius

    def distance_to(self, point):
        """Returns the euclidian distance from the agent to a point"""
        return np.linalg.norm(self.pos - point)

    def vel_ang_ok(right_ang, left_ang, vel_ang):
        """ Checks if the velocity is valid given right and left boundaries """

        if right_ang > left_ang:
            # The velocity need to be larger than left and smaller than right
            return left_ang < vel_ang < right_ang

        if right_ang <= vel_ang <= left_ang:
            return False
        else:
            return True

    def get_bound_ang(self, agent_b):
        """ Returns the left and right boundries given agent a traveling and agent b being and obstacle. """

        vec_ab = agent_b.pos - self.pos

        theta = atan2(vec_ab[1], vec_ab[0])
        alpha = tan((self.r + agent_b.r)/np.linalg.norm(vec_ab))

        print("Theta: ", theta)
        print("Aplha: ", alpha)

        ang_right = theta - alpha
        ang_left = theta + alpha

        return ang_right, ang_left

    def get_avoidance_vels(self, neighbors):
        """ Returns the velocities for all neighbors that avoids collitions"""


        for neighbor in neighbors:

            angs = [self.get_bound_ang(neighbor)]

            # try different vewlocities from the solfjäder that does not colide with [angs]




        vels = []

        return vels

    def find_best_vel(self, neighbors):
        """ Returns the best velocity given the neighbors and goal vel"""

        vels = self.get_avoidance_vels(neighbors)

        # Calcultate the best vel given goal_pos


        return #new_vel




# Test code
hej = Agent(np.array([3, 1]), np.array([10, 10]), 2)
hej2 = Agent(np.array([5, 2]), np.array([10, 10]), 2)

hej.get_bound_ang(hej2)