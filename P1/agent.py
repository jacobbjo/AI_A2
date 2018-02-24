import numpy as np
from math import atan2

class Agent(object):

    def __init__(self, start_pos, goal_pos, start_vel = np.zeros(2)):
        self.pos = start_pos
        self.goal = goal_pos
        self.vel = start_vel

    def distance_to(self, point):
        """Returns the euclidian distance from the agent to a point"""
        return np.linalg.norm(self.pos - point)


    def find_best_vel(self, neighbors):
        """ Returns the best velocity given the neighbors and goal vel"""

        vels = self.get_avoidance_vels(neighbors)

        # Calcultate the best vel given goal_pos




        return #new_vel



    def get_bound_ang(self, agent_b):
        """ Returns the left and right boundries given agent a traveling and agent b being and obstacle. """
        vec_ab = agent_b.pos - self.pos
        theta = atan2(vec_ab[1], vec_ab[0])
        print(theta)

        point_a_bvel = self.pos + agent_b.vel  # Add b's velocity to the location of a



        return #ang_right, ang_left




    def get_avoidance_vels(self, neighbors):
        """ Returns the velocities for all neighborsthat avoids collitions"""


        for neighbor in neighbors:

            angs = [self.get_bound_ang(neighbor)]

            # try different vewlocities from the solfjäder that does not colide with [angs]




        vels = []

        return vels




# Test code
hej = Agent([3, 1], [10, 10])
get_bound_ang()