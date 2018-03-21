import numpy as np
from math import *
from Common.state import Point
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


        new_point = self.is_obs_on_path(the_map)
        while new_point is not None:
            self.poi.insert(0, Point(new_point, -1))
            new_point = self.is_obs_on_path(the_map)

        vel_needed = self.poi[0].xy - self.pos

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

    def is_obs_on_path(self, the_map):
        """Returns the closest obstacle on the straight path to the first poi, None if the path is clear
        """
        intersec_point = []
        intersec_vert_ind = []
        intersec_obstacles = []

        path_line = np.array([self.pos, self.poi[0].xy])

        for obstacle in the_map.obstacles:
            for i in range(len(obstacle.vertices)):
                if obstacle.lines_intersect(obstacle.vertices[i], obstacle.vertices[i - 1], self.pos, self.poi[0].xy):
                    obs_edge = np.array([obstacle.vertices[i], obstacle.vertices[i - 1]])

                    # Find the intersecting point and save it, save the edge
                    intersec_point.append(self.find_intersecting_point(path_line, obs_edge))
                    #intersec_edges.append(obs_edg)
                    intersec_vert_ind.append(i)
                    intersec_obstacles.append(obstacle)

        # Finds the edge closest to the current position
        if len(intersec_obstacles) == 0:
            return None

        min_distance = float("infinity")
        close_edge_vert_index = -1
        for point_index, point in enumerate(intersec_point):
            distance = np.linalg.norm(point-self.pos)
            if distance < min_distance:
                min_distance = distance
                close_edge_vert_index = point_index

        # Finds the corner of the edge closest to the poi-point


        #close_index = intersec_vert_ind[close_edge_vert_index]
        col_obstacle = intersec_obstacles[close_edge_vert_index]
        point_on_col_edge_index = intersec_vert_ind[close_edge_vert_index]

        close_point = col_obstacle.vertices[point_on_col_edge_index]
        col_vert = col_obstacle.vertices[point_on_col_edge_index-1]
        other_vert = col_obstacle.vertices[(point_on_col_edge_index+1)%len(col_obstacle.vertices)]

        close_point2 = col_obstacle.vertices[point_on_col_edge_index-1]
        col_vert2 = col_obstacle.vertices[point_on_col_edge_index]
        other_vert2 = col_obstacle.vertices[point_on_col_edge_index - 2]

        proj_point = (col_vert - close_point) + other_vert
        new_point = close_point + ((close_point - proj_point)/np.linalg.norm(close_point - proj_point)) * self.r

        proj_point2 = (col_vert2 - close_point2) + other_vert2
        new_point2 = close_point2 + ((close_point2 - proj_point2)/np.linalg.norm(close_point2 - proj_point2)) * self.r

        if np.linalg.norm(col_obstacle.vertices[point_on_col_edge_index] - self.poi[0].xy) < np.linalg.norm(col_obstacle.vertices[point_on_col_edge_index-1] - self.poi[0].xy):
            if the_map.valid_point(new_point):
                print("First point, ", new_point)
                return new_point
            else:
                print("second point, ", new_point2)
                return new_point2
        else:
            if the_map.valid_point(new_point2):
                print("First point", new_point)
                return new_point2
            else:
                print("second point, ", new_point2)
                return new_point

    def find_intersecting_point(self, path_line, obs_edge):
        x_mat_line1, x_mat_line2, y_mat_line1, y_mat_line2  = np.ones(path_line.shape), np.ones(path_line.shape),\
                                                              np.ones(path_line.shape), np.ones(path_line.shape)
        x_mat_line1[:, 0] = path_line[:, 0]
        x_mat_line2[:, 0] = obs_edge[:, 0]
        y_mat_line1[:, 0] = path_line[:, 1]
        y_mat_line2[:, 0] = obs_edge[:, 1]

        div_det = np.linalg.det(
            np.array([[np.linalg.det(x_mat_line1), np.linalg.det(y_mat_line1)], [np.linalg.det(x_mat_line2),
                                                                                 np.linalg.det(y_mat_line2)]]))
        x_det = np.linalg.det(
            np.array([[np.linalg.det(path_line), np.linalg.det(x_mat_line1)], [np.linalg.det(obs_edge),
                                                                               np.linalg.det(x_mat_line2)]]))
        y_det = np.linalg.det(
            np.array([[np.linalg.det(path_line), np.linalg.det(y_mat_line1)], [np.linalg.det(obs_edge),
                                                                               np.linalg.det(y_mat_line2)]]))
        return np.array([x_det/div_det, y_det/div_det])


#point1 = np.array([0, 0])
#point2 = np.array([3, 3])
#int_point1 = np.array([1, 3])
#int_point2 = np.array([3, 1])
#no_int_point1 = np.array([2, 3])
#no_int_point2 = np.array([1, 4])
#
#path_line = np.array([point1, point2])
#obs_edge = np.array([int_point1, int_point2])
#
#x_mat_line1, x_mat_line2, y_mat_line1, y_mat_line2 = np.ones(path_line.shape), np.ones(path_line.shape), \
#                                                     np.ones(path_line.shape), np.ones(path_line.shape)
#
#x_mat_line1[:, 0] = path_line[:, 0]
#x_mat_line2[:, 0] = obs_edge[:, 0]
#y_mat_line1[:, 0] = path_line[:, 1]
#y_mat_line2[:, 0] = obs_edge[:, 1]
#
#x_diff = np.array([path_line[0, 0] - path_line[1, 0], obs_edge[0, 0] - obs_edge[1, 0]])
#y_diff = np.array([path_line[0, 1] - path_line[1, 1], obs_edge[0, 1] - obs_edge[1, 1]])
#
#diff_mat = np.array([x_diff, y_diff])
#print("diff ", np.linalg.det(diff_mat))
#
#div_det = np.linalg.det(np.array([[np.linalg.det(x_mat_line1), np.linalg.det(y_mat_line1)], [np.linalg.det(x_mat_line2),
#                                                                                         np.linalg.det(y_mat_line2)]]))
#x_det = np.linalg.det(np.array([[np.linalg.det(path_line), np.linalg.det(x_mat_line1)], [np.linalg.det(obs_edge),
#                                                                                         np.linalg.det(x_mat_line2)]]))
#y_det = np.linalg.det(np.array([[np.linalg.det(path_line), np.linalg.det(y_mat_line1)], [np.linalg.det(obs_edge),
#                                                                                         np.linalg.det(y_mat_line2)]]))
#
#
#print(x_det/div_det)
#print(y_det/div_det)
#
#
#print(x_mat_line1)
#print(y_mat_line2)


