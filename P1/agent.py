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


        """ Checks if the velocity is allowed given right and left boundaries """

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
        #print(alpha)
        # Angles from the x-axis to the boundaries
        ang_right = theta - alpha
        ang_left = theta + alpha

        return [ang_right, ang_left]

    def vel_ang_ok_neigh(self, test_vel, neighbors, bound_angs):
        """ Returns if the tested velocity is valid given all the neighbors bounding angles"""

        for i in range(len(neighbors)):
            diff_vel = test_vel - neighbors[i].vel

            if not self.vel_ang_ok(bound_angs[i][0], bound_angs[i][1], atan2(diff_vel[1], diff_vel[0])):
                return False

        return True

    def get_avoidance_vels(self, neighbors, v_max):
        """ Returns the velocities for all neighbors that avoids collitions"""

        # Parameters to control the amount of tested velocities
        rad_step = v_max/20
        ang_step = np.pi/30

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
            return [self.v_des]

        # Finds all the possible velocities
        pos_vels = []
        for ang in np.arange(ang_des-lim_ang/2, ang_des+lim_ang/2, ang_step):
            for rad in np.arange(0, lim_rad + rad_step, rad_step):
                test_vel = np.array([rad * cos(ang), rad * sin(ang)])

                if self.vel_ang_ok_neigh(test_vel, neighbors, bound_angs):
                    pos_vels.append(test_vel)
        print("lol")
        print(len(pos_vels))
        print(pos_vels)
        return pos_vels

    def find_best_vel(self, neighbors, v_max):
        """ Returns the best velocity given the neighbors and goal vel"""

        self.update_des_vel(v_max)
        pos_vels = self.get_avoidance_vels(neighbors, v_max)
        #print(len(vels))
        min_vel_distance = float("infinity")
        best_vel = np.zeros(2)

        for pos_vel in pos_vels:
            vel_distance = np.linalg.norm(self.v_des - pos_vel)
            if vel_distance < min_vel_distance:
                best_vel = pos_vel
                min_vel_distance = vel_distance

        return best_vel

    def update_des_vel(self, vmax):
        """ Updates the desired velocity to get to the goal"""
        vel_needed = self.goal - self.pos

        if np.linalg.norm(vel_needed) > vmax:
            vel_needed = vel_needed/ np.linalg.norm(vel_needed)
            vel_needed *= vmax
        self.v_des = vel_needed


# Test code
def main():
    hej = Agent(np.array([1, 1]), np.array([10, 10]), 0.5)
    hej2 = Agent(np.array([2, 3]), np.array([10, 10]), 0.5, np.array([1, -1]))

    best = hej.find_best_vel([hej2], 1.2)
    print("Lenght best: ", np.linalg.norm(best))
    print(best)

    # hej.get_bound_ang(hej2)


main()