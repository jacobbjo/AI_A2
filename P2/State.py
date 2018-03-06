import numpy as np


class Point(object):

    def __init__(self, xy, ind):
        self.xy = xy
        self.index = ind

class Route(object):

    def __init__(self, start, goal, route_list):
        self.start = start
        self.goal = goal
        self.route = route_list
        self.tot_distance = self.calc_tot_distance()
        self.num_points = len(self.route)

    def calc_tot_distance(self):
        tot_distance = 0
        if len(self.route) > 0:
            tot_distance += np.linalg.norm(self.route[0].xy - self.start)
            tot_distance += np.linalg.norm(self.goal - self.route[len(self.route)-1].xy)
            for i in range(len(self.route)-1):
                tot_distance += np.linalg.norm(self.route[i+1].xy - self.route[i].xy)
        return tot_distance

    def two_opt(self):
        for i in range(self.num_points - 2):
            for j in range(i+2, self.num_points-1):
                point1 = self.route[i]
                point2 = self.route[i+1]
                point3 = self.route[j]
                point4 = self.route[j+1]
                original_dist = np.linalg.norm(point1.xy - point2.xy) + np.linalg.norm(point3.xy - point4.xy)
                changed_dist = np.linalg.norm(point1.xy - point3.xy) + np.linalg.norm(point2.xy - point4.xy)
                if changed_dist < original_dist:
                    self.route = self.change_route(i, j)

    def change_route(self, ind_to_change1, ind_to_change2):
        #new_route = self.route[:ind_to_change1]
        #new_route.append(self.route[ind_to_change1+1:ind_to_change2].reverse())
        #new_route.append(self.route[ind_to_change2+1:])
        #self.route = new_route
        self.route[ind_to_change1+1:ind_to_change2] = self.route[ind_to_change1+1:ind_to_change2].reverse()

class State(object):

    def __init__(self, routes, v_max=1.0):
        """ Routes are a list with route objects"""
        self.routes = routes
        self.routes_distance = self.calc_routes_distance()
        self.max_time = self.calc_tot_time()
        self.agent_v_max = v_max

    def calc_routes_distance(self):
        tot_distance = 0
        for route in self.routes:
            tot_distance = route.tot_distance
        return tot_distance

    def calc_tot_time(self):
        max_time = -float("infinity")
        for route in self.routes:
            time = route.tot_distance * self.agent_v_max
            if time > max_time:
                max_time = time
        return max_time

    def add_route(self, new_route):
        self.routes.append(new_route)
        self.routes_distance += new_route.tot_distance
        if new_route.tot_distance * self.agent_v_max > self.max_time:
            self.max_time = new_route.tot_distance

    def findNeighborhood(self):
        # Vad vill vi ha här? Two opt? Ta en punkt från en route till en annan
        # To change a point from one route to another.
            # Plocka ut två random routes. (Om det blir samma route, gör 2-opt)
            # Byt plats på en random point från den första routen till den andra.
            # Ett nytt neighborhood hittat.

        return []
