import numpy as np
from math import cos, sin, tan, atan2


class Minion(object):

    def __init__(self, start_pos, formation_start, leader_formation_start, leader_trajectory_goal):
        self.pos = start_pos
        self.formation_start = formation_start
        self.offset = np.array([formation_start[0] - leader_formation_start[0],
                                formation_start[1] - leader_formation_start[1]])
        self.pos_hist = [self.pos] # keep track of all positions for later visualization
        self.v_des = np.zeros(2)
        self.vel = np.zeros(2)
        self.r = 0.5


    def at_goal(self, the_map):
        """ Returns whether the minion is at its goal or not"""
        rot_ang = the_map.leader_theta[-1] - np.pi / 2
        rotation_matrix = np.array([[cos(rot_ang), -sin(rot_ang)], [sin(rot_ang), cos(rot_ang)]])
        goal = the_map.leader_positions[-1] + np.matmul(rotation_matrix, self.offset)
        if np.linalg.norm(goal - self.pos) < the_map.vehicle_v_max * the_map.vehicle_dt:
            return True
        return False

    def at_start(self, the_map):
        """ Returns whether the minion is at its goal or not"""
        rot_ang = the_map.leader_theta[0] - np.pi / 2
        rotation_matrix = np.array([[cos(rot_ang), -sin(rot_ang)], [sin(rot_ang), cos(rot_ang)]])
        start = the_map.leader_positions[0] + np.matmul(rotation_matrix, self.offset)

        if np.linalg.norm(start - self.pos) < the_map.vehicle_v_max * the_map.vehicle_dt:
            return True
        return False

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
        """ Returns the left and right boundaries given agent a traveling and agent b being an obstacle. """

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

    def find_best_vel(self, aim_point, minions, neighbor_limit, the_map):
        """ Returns the best velocity given the neighbors and goal vel"""

        neighbors = self.get_neighbors(minions, neighbor_limit)

        self.update_des_vel(aim_point, the_map)
        pos_vels = self.get_avoidance_vels(neighbors, the_map)

        min_vel_distance = float("infinity")
        best_vel = np.zeros(2)

        for pos_vel in pos_vels:
            vel_distance = np.linalg.norm(self.v_des - pos_vel)
            if vel_distance < min_vel_distance:
                best_vel = pos_vel
                min_vel_distance = vel_distance
        return best_vel

    def update_des_vel(self, aim_point, the_map):
        """ Updates the desired velocity to get to the next poi"""
        v_max = the_map.vehicle_v_max
        dt = the_map.vehicle_dt

        vel_needed = aim_point - self.pos

        #if np.linalg.norm(vel_needed) > v_max:
        vel_needed = vel_needed / np.linalg.norm(vel_needed)
        vel_needed *= v_max
        self.v_des = vel_needed

    def get_neighbors(self, agent_list, limit):
        """
        :param agent: The agent to whom the neighbors will be returned
        :param agent_list: list of all agents
        :param limit: The maximum distance which are considered neighborhood
        :return: list of all the neighbors to agent
        """
        neighbors = []

        for agent in agent_list:
            if agent == self:
                continue
            if np.linalg.norm(agent.pos - self.pos) < limit:
                neighbors.append(agent)

        return neighbors

    def move(self, leader_pos, leader_theta, minions, limit, the_map):

        rot_ang = leader_theta - np.pi/2
        rotation_matrix = np.array([[cos(rot_ang), -sin(rot_ang)], [sin(rot_ang), cos(rot_ang)]])
        aim_point = leader_pos + np.matmul(rotation_matrix, self.offset)
        #print(aim_point)
        if not np.linalg.norm(aim_point - self.pos) < the_map.vehicle_v_max * the_map.vehicle_dt:


            new_vel = self.find_best_vel(aim_point, minions, limit, the_map)
            self.vel = new_vel

            self.pos = self.pos + (self.vel * the_map.vehicle_dt)
        self.save_pos()




