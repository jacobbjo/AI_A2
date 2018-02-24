import numpy as np
from math import *


class Agent(object):

    def __init__(self, start_pos, goal_pos, radius, start_vel = np.zeros(2)):
        self.pos = start_pos
        self.goal = goal_pos
        self.vel = start_vel
        self.v_des = np.zeros(2)
        self.r = radius

    def distance_to(self, point):
        """Returns the euclidian distance from the agent to a point"""
        return np.linalg.norm(self.pos - point)

    def vel_ang_ok(self, right_ang, left_ang, vel_ang):
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

        ang_right = theta - alpha
        ang_left = theta + alpha

        return ang_right, ang_left

    def get_avoidance_vels(self, neighbors, v_max):
        """ Returns the velocities for all neighbors that avoids collitions"""

        ang_des = atan2(self.v_des[1], self.v_des[0])

        rad_step = v_max/10
        ang_step = np.pi/20

        bound_angs = [] # Not allowed angles

        # Calculates the bounds for each neighbor
        for neighbor in neighbors:
            bound_right, bound_left = self.get_bound_ang(neighbor)
            bound_angs.append([bound_right, bound_left])

        # Checks if the desired velocity is OK
        des_vel_ok = True
        for i in range(len(neighbors)):
            diff_vel = self.v_des - neighbors[i].vel
            if not self.vel_ang_ok(bound_angs[i][0], bound_angs[i][1], atan2(diff_vel[1], diff_vel[0])):
                print(self.vel_ang_ok(bound_angs[i][0], bound_angs[i][1], atan2(diff_vel[1], diff_vel[0])))
                des_vel_ok = False
                break
        if des_vel_ok:
           return [self.v_des]

        # Finds all the possible velocities
        pos_vels = []
        for ang in np.arange(ang_des-np.pi/2, ang_des+np.pi/2, ang_step):

            for rad in np.arange(0, v_max, rad_step):
                test_vel = np.array([rad * cos(ang), rad * sin(ang)])
                vel_ok = True
                for i in range(len(neighbors)):
                    diff_vel = test_vel - neighbors[i].vel

                    if not self.vel_ang_ok(bound_angs[i][0], bound_angs[i][1], atan2(diff_vel[1], diff_vel[0])):
                        vel_ok = False
                        break

                if vel_ok:
                    pos_vels.append(test_vel)

        return pos_vels


    def find_best_vel(self, neighbors, v_max):
        """ Returns the best velocity given the neighbors and goal vel"""

        self.update_des_vel(v_max)

        vels = self.get_avoidance_vels(neighbors, v_max)
        print(vels)

        min_vel_distance = float("infinity")
        best_vel = None

        for pos_vel in vels:
            vel_distance = np.linalg.norm(self.v_des - pos_vel)
            if vel_distance < min_vel_distance:
                best_vel = pos_vel
                min_vel_distance = vel_distance

        return best_vel


    def update_des_vel(self, vmax):
        vel_needed = self.goal - self.pos

        if np.linalg.norm(vel_needed) > vmax:
            print(vel_needed)
            print(np.linalg.norm(vel_needed))
            vel_needed = vel_needed/ np.linalg.norm(vel_needed)
            vel_needed *= vmax
            print(vel_needed)
        self.v_des = vel_needed




# Test code
hej = Agent(np.array([1, 1]), np.array([10, 10]), 0.5)
hej2 = Agent(np.array([2, 3]), np.array([10, 10]), 0.5)

print(hej.find_best_vel([hej2], 1.2))

#hej.get_bound_ang(hej2)