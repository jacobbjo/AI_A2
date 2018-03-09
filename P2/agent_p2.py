import numpy as np
from math import *
import matplotlib.pyplot as plt
#from help_functions import *


class Agent(object):

    def __init__(self, in_index, start_pos, goal_pos, in_poi, radius, start_vel = np.zeros(2)):
        self.pos = start_pos.xy
        self.goal = goal_pos
        self.vel = start_vel
        self.v_des = np.zeros(2)
        self.r = radius
        self.index = in_index
        self.poi = in_poi
        self.pos_hist = [] # keep track of all positions for later visualization
        self.is_moving = True
        self.poi.append(self.goal)

    def distance_to(self, point):
        """Returns the euclidian distance from the agent to a point"""

        return np.linalg.norm(self.pos - point)

    def vel_ang_ok(self, right_ang, left_ang, vel_ang):
        """ Checks if the velocity is allowed given right and left boundaries """

        if right_ang < 0:
            right_ang += (2*np.pi)

        if left_ang < 0:
            left_ang += (2*np.pi)

        if vel_ang < 0:
            vel_ang += (2*np.pi)

        if right_ang > left_ang:
            # The velocity need to be larger than left and smaller than right
            return left_ang < vel_ang < right_ang

        # If/else to prevent from false negative when right_ang < left_ang < vel_ang
        if right_ang <= vel_ang <= left_ang:
            return False
        else:
            return True

    def get_bound_ang(self, agent_b):
        """ Returns the left and right boundaries given agent a traveling and agent b being and obstacle. """

        rad_exp = self.r/10 # How close to each other the agents travel

        # The line between self and agent b
        vec_ab = agent_b.pos - self.pos
        rad_ab = self.r + agent_b.r + rad_exp
        dist = np.linalg.norm(vec_ab)
        if dist <= rad_ab:
            dist = rad_ab

        theta = atan2(vec_ab[1], vec_ab[0])  # The angle from the x-axis to vec_ab
        alpha = tan(rad_ab/dist)  # The angle from vec_ab to the boundaries

        # Angles from the x-axis to the boundaries
        ang_right = theta - alpha
        ang_left = theta + alpha

        return [ang_right, ang_left]

    def vel_ang_ok_neigh(self, test_vel, neighbors, bound_angs):
        """ Returns if the tested velocity is valid given all the neighbors bounding angles"""

        for i in range(len(neighbors)):

            trans_b_a = self.pos + 0.5 *(neighbors[i].vel + self.vel)  # <----------- change this to change model
            diff_vel = test_vel + self.pos - trans_b_a

            #diff_vel = test_vel - neighbors[i].vel

            if not self.vel_ang_ok(bound_angs[i][0], bound_angs[i][1], atan2(diff_vel[1], diff_vel[0])):
                return False

        return True

    def get_avoidance_vels(self, neighbors, the_map):
        """ Returns the velocities for all neighbors that avoids collitions"""
        v_max = the_map.vehicle_v_max
        dt = the_map.vehicle_dt

        # Parameters to control the amount of tested velocities
        rad_step = v_max/5
        ang_step = np.pi/8

        # Search limit for the angle
        lim_ang = 2 * np.pi  # Maximum 2π
        lim_rad = v_max

        # The angle from the x-axis to the desired velocity (to get to the goal)
        ang_des = atan2(self.v_des[1], self.v_des[0])

        bound_angs = []  # Pairs of angles where velocities between them is not allowed

        # Calculates the bounds for each neighbor
        for neighbor in neighbors:
            bound_angs.append(self.get_bound_ang(neighbor))

        if self.vel_ang_ok_neigh(self.v_des, neighbors, bound_angs):
            if the_map.valid_point(self.pos + (self.v_des * dt )):
                return [self.v_des]

        # Finds all the possible velocities
        pos_vels = []
        for ang in np.arange(ang_des-lim_ang/2, ang_des+lim_ang/2, ang_step):
            for rad in np.arange(0, (lim_rad + rad_step), rad_step):
                test_vel = np.array([rad * cos(ang), rad * sin(ang)])

                if self.vel_ang_ok_neigh(test_vel, neighbors, bound_angs):
                    if the_map.valid_point(self.pos + (test_vel * dt)):
                        pos_vels.append(test_vel)

        return pos_vels

    def save_pos(self):
        self.pos_hist.append(np.copy(self.pos))

    def find_best_vel(self, agents, neighbor_limit, the_map):
        """ Returns the best velocity given the neighbors and goal vel"""

        neighbors = self.get_neighbors(agents, neighbor_limit)

        self.update_des_vel(the_map)
        pos_vels = self.get_avoidance_vels(neighbors, the_map)

        # REMOVE THIS BEFORE FINAL!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
        #if len(pos_vels) == 0:
            #print("Agent ", str(self.index), " har inget val")

        min_vel_distance = float("infinity")
        best_vel = np.zeros(2)

        for pos_vel in pos_vels:
            vel_distance = np.linalg.norm(self.v_des - pos_vel)
            if vel_distance < min_vel_distance:
                best_vel = pos_vel
                min_vel_distance = vel_distance
        return best_vel

    def update_des_vel(self, the_map):
        """ Updates the desired velocity to get to the next poi"""
        v_max = the_map.vehicle_v_max
        dt = the_map.vehicle_dt
        rot_ang = pi/8
        rot_mat = np.array([[np.cos(rot_ang), -np.sin(rot_ang)], [np.sin(rot_ang), np.cos(rot_ang)]])


        vel_needed = self.poi[0].xy - self.pos

        #if np.linalg.norm(vel_needed) > v_max:
        vel_needed = vel_needed / np.linalg.norm(vel_needed)
        vel_needed *= v_max

        while True:
            if the_map.valid_point(self.pos + (vel_needed * dt)):
                self.v_des = vel_needed
                break
            else:
                print("invalid point: ", self.pos + (vel_needed * dt))
                #print("rotating, vel_needed before: ", vel_needed)
                vel_needed = np.matmul(rot_mat,  vel_needed)
                print(vel_needed)
                #print("rotating, vel_needed after: ", vel_needed)






    def get_neighbors(self, agent_list, limit):
        """
        :param agent: The agent to whom the neighbors will be returned
        :param agent_list: list of all agents
        :param limit: The maximum distance which are considered neighborhood
        :return: list of all the neighbors to agent
        """

        neighbors = []

        for agent in agent_list:
            if agent.index == self.index:
                continue
            if np.linalg.norm(agent.pos - self.pos) < limit:
                neighbors.append(agent)

        return neighbors

    def check_route_status(self, limit):
        if np.linalg.norm(self.poi[0].xy - self.pos) < limit:
            print("Found point! ind: ", self.index)
            self.poi.pop(0)

        if len(self.poi) == 0:
            self.is_moving = False
