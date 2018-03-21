""" Originally from A1 by Jacob Björkman and Alex Hermansson"""


import numpy as np
import json
import matplotlib.pyplot as plt

class Polygon:
    """Represents a polygon """

    def __init__(self, vertices):
        self.vertices = self.to_np_vertices(vertices)
        self.x_min = 10 ** 8
        self.x_max = 0
        self.y_min = 10 ** 8
        self.y_max = 0

        self.find_limmits()

    def to_np_vertices(self, vertices):
        """Converts every vertex in list to ndarray form."""

        np_vertices = []
        for vertex in vertices:
            np_vertices.append(np.array(vertex))

        return np_vertices

    def find_limmits(self):
        """Finds the bounding box for the polygon"""

        for vertice in self.vertices:
            if vertice[0] > self.x_max:
                self.x_max = vertice[0]
            if vertice[0] < self.x_min:
                self.x_min = vertice[0]

            if vertice[1] > self.y_max:
                self.y_max = vertice[1]
            if vertice[1] < self.y_min:
                self.y_min = vertice[1]

    def contain_point(self, point):
        """Determines if the given point is in the polygon or not"""

        # If the point is within the bounding box we need to do a raycast to be more precise
        if self.point_in_box(point):
            return self.point_in_polygon(point)

        return False

    def point_in_box(self, point):
        """Determines if the given point is in the polygons outer boundries or not"""

        # SHOULD IT BE <= ????????????????????????????????????????????????????????????????
        return self.x_min < point[0] < self.x_max and self.y_min < point[1] < self.y_max

    def point_in_polygon(self, point):
        """Determines if the point is in the actual polygon, uses ray cast"""

        # Cast a ray from the given point to ray_end, a bit outside of the boundries
        ray_end = [self.x_max + 1, self.y_max + 1]

        intersections = 0

        for i in range(len(self.vertices)):
            if self.lines_intersect(self.vertices[i], self.vertices[i - 1], point, ray_end):
                intersections += 1

        return intersections % 2 == 1

    def lines_intersect(self, p1, q1, p2, q2):
        """
        Determines whether the line p1-q1 intersects the line p2-q2

        https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/

        """

        o1 = self.orientation(p1, q1, p2)
        o2 = self.orientation(p1, q1, q2)
        o3 = self.orientation(p2, q2, p1)
        o4 = self.orientation(p2, q2, q1)

        if o1 != o2 and o3 != o4:
            return True

        return False

    def orientation(self, p, q, r):
        """
        Determines if walking through the tree points is done clock- or contrerclockwise

        https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/

        """

        val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])

        if val == 0:
            return 0  # colinear

        elif val > 0:
            return 1  # clockwise

        else:
            return 2  # counterclockwise

    def plot(self, col):
        """Plots the polygon"""

        for i in range(len(self.vertices) - 1):
            # print(self.vertices[i])
            v1 = self.vertices[i]
            v2 = self.vertices[i + 1]
            plt.plot([v1[0], v2[0]], [v1[1], v2[1]], color=col)

        plt.plot([v2[0], self.vertices[0][0]],
                 [v2[1], self.vertices[0][1]], color=col)


class Problem:
    """Imports a JSON file and contains the retrived problem data"""

    def __init__(self, filename):
        self.data = self.read_file(filename)

        self.obstacles = []

        for key, value in self.data.items():
            if "obstacle" in key:
                self.obstacles.append(Polygon(value))

        self.bounding_polygon = Polygon(self.data["bounding_polygon"])
        self.goal_positions = np.array(self.data["goal_positions"])
        self.points_of_interest = self.data["points_of_interest"]
        self.sensor_range = self.data["sensor_range"]
        self.start_positions = np.array(self.data["start_positions"])
        self.vehicle_L = self.data["vehicle_L"]
        self.vehicle_a_max = self.data["vehicle_a_max"]
        self.vehicle_dt = self.data["vehicle_dt"]
        self.vehicle_omega_max = self.data["vehicle_omega_max"]
        self.vehicle_phi_max = self.data["vehicle_phi_max"]
        self.vehicle_t = self.data["vehicle_t"]
        self.vehicle_v_max = self.data["vehicle_v_max"]


    def read_file(self, filename):
        """Returns the data as a dictionary """

        if filename:
            with open(filename, 'r') as file:
                return json.load(file)

        return None

    def valid_point(self, point):
        """Determines if the point is within the outer polygon and not within an obstacle"""

        if self.bounding_polygon.contain_point(point):
            for obstacle in self.obstacles:
                if obstacle.contain_point(point):
                    return False
            return True
        return False

    def plot_map(self):
        """Plots the border polygon and the obstacle polygons together
        with start and goal positions and velocities."""

        plt.axis("equal")

        self.bounding_polygon.plot("b")

        for obstacle in self.obstacles:
            obstacle.plot("r")


def plot_vector(start, end):
    """Takes two points as input and plot the vector between them."""

    x_diff = end[0] - start[0]
    y_diff = end[1] - start[1]
    x_vec = [start[0], start[0] + x_diff]
    y_vec = [start[1], start[1] + y_diff]
    plt.plot(x_vec, y_vec)


def main():
    json_in = Problem("P23.json")
    print("vehicle_L: ", json_in.vehicle_L)
    print("vehicle_a_max: ", json_in.vehicle_a_max)
    json_in.plot_map()

    p = [5, 5] # valid
    # p = [9, 7] #not valid
    # p = [30, 40] #not valid

    # plt.plot([p[0]],[p[1]], "ro")

    # print("Valid point?:")
    print(json_in.valid_point(p))

#main()