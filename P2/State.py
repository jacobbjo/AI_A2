import numpy as np
import random
import copy


class Point(object):

    def __init__(self, xy, ind):
        self.xy = xy
        self.index = ind

    def __str__(self):
        return str(self.index)

    def __repr__(self):
        return self.__str__()

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
            tot_distance += np.linalg.norm(self.route[0].xy - self.start.xy)
            tot_distance += np.linalg.norm(self.goal.xy - self.route[len(self.route)-1].xy)
            for i in range(len(self.route)-1):
                tot_distance += np.linalg.norm(self.route[i+1].xy - self.route[i].xy)
        return tot_distance

    def two_opt(self):
        if self.num_points < 4 or self.route is None:
            return
        #print(self.route[1])
        #print(self.num_points)

        # we have to consider the start and goal when calculating the two opt
        complete_route = [self.start] + self.route + [self.goal]

        for i in range(len(complete_route) - 2):
            for j in range(i+2, len(complete_route) - 1):

                point1 = complete_route[i]
                point2 = complete_route[i+1]
                point3 = complete_route[j]
                point4 = complete_route[j+1]

                original_dist = np.linalg.norm(point2.xy - point1.xy) + np.linalg.norm(point4.xy - point3.xy)
                changed_dist = np.linalg.norm(point3.xy - point1.xy) + np.linalg.norm(point4.xy - point2.xy)

                if changed_dist < original_dist:
                    complete_route[i + 1:j + 1] = reversed(complete_route[i + 1:j + 1])

        self.route = complete_route[1:-1]


    def change_routes(self, otherRoute):
        if self == otherRoute:
            self.two_opt()
        else:
            random_point = self.route[random.randint(0, self.num_points-1)]
            position = random.randint(0, otherRoute.num_points)
            self.remove_point(random_point)
            otherRoute.add_point(random_point, position)

    def add_point(self, point_to_add, position):
        self.route.insert(position, point_to_add)
        self.num_points += 1
        self.tot_distance = self.calc_tot_distance()

    def remove_point(self, point_to_remove):
        self.route.remove(point_to_remove)
        self.num_points -= 1
        self.tot_distance = self.calc_tot_distance()

    def __str__(self):
        return str(self.route)

    def __repr__(self):
        return str(self.route)

class State(object):

    def __init__(self, routes, v_max=1.0):
        """ Routes are a list with route objects"""
        self.routes = routes
        self.routes_distance = self.calc_routes_distance()
        self.agent_v_max = v_max
        self.max_time = self.calc_tot_time()

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

    def find_neighborhood(self):
        neighborhood = []
        neighborhood_size = 1000
        for i in range(neighborhood_size):
            neighbor = self.find_neighbor()
            if neighbor != None:
                neighborhood.append(neighbor)
        print(self.max_time)
        for neighbor in neighborhood:
            print(neighbor.max_time)
        return neighborhood

    def find_neighbor(self):
        neighbor = copy.deepcopy(self)
        index1 = random.randint(0, len(neighbor.routes)-1)
        index2 = random.randint(0, len(neighbor.routes)-1)
        route1 = neighbor.routes[index1]
        route2 = neighbor.routes[index2]
        if route1.num_points == 0:
            return None
        route1.change_routes(route2)
        neighbor.max_time = neighbor.calc_tot_time()
        return neighbor

