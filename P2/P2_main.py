from importJSON2 import Problem
import numpy as np
import matplotlib.pyplot as plt
from State import *


def calcDistance(point1, point2):
    return np.linalg.norm(point2-point1)


def assign_points(points, starts, goals, v_max):
    routes = []
    for i in range(starts.shape[0]):
        routes.append([])

    for i in range(len(points)):
        min_distance = float("infinity")
        min_index = -1
        for j in range(starts.shape[0]):
            distance = min(calcDistance(points[i].xy, starts[j]), calcDistance(points[i].xy, goals[j]))
            if distance < min_distance:
                min_distance = distance
                min_index = j
        routes[min_index].append(points[i])
    state = State([], v_max)
    for i in range(len(routes)):
        route = routes[i]
        route_object = Route(starts[i], goals[i], route)
        state.add_route(route_object)

    return state


def create_points(point_list):
    points = []
    for i in range(len(point_list)):
        new_point = Point(point_list[i], i)
        points.append(new_point)
    return points

def main():

    # Var ska ska skiftet i routerna göras och tabuerna hållas koll på?
    # Hur ska skiftena göras?
    # Vilka skiften ska göras?

    the_map = Problem("P22.json")
    points = create_points(the_map.points_of_interest)
    starts = the_map.start_positions
    goals = the_map.goal_positions
    dt = the_map.vehicle_dt
    v_max = the_map.vehicle_v_max
    # Assign each point the the agent with the closest start or goalPoint
    init_state = assign_points(points, starts, goals, v_max)
    init_routes = init_state.routes

    init_routes[2].two_opt()

    # Show the point assignments
    colors = createColorDictDist()
    for agent_index in range(len(init_routes)):
        color = colors[agent_index+1]
        for i in range(init_routes[agent_index].num_points):
            plt.plot(init_routes[agent_index].route[i].xy[0], init_routes[agent_index].route[i].xy[1], "o", c=color)

        plt.plot(starts[agent_index][0], starts[agent_index][1], "x", c=color)
        plt.plot(goals[agent_index][0], goals[agent_index][1], "x", c=color)

    #for agent_index in range(len(init_routes)):
    #    color = colors[agent_index+1]
    #    for i in range(len(init_routes[agent_index])):
    #        plt.plot(init_routes[agent_index][i][0], init_routes[agent_index][i][1], "o", c=color)
    #
    #    plt.plot(starts[agent_index][0], starts[agent_index][1], "x", c=color)
    #    plt.plot(goals[agent_index][0], goals[agent_index][1], "x", c=color)

    plt.show()

def createColorDictDist():
    # It is 29 districts
    colorDict = {}
    colorDict[1] = "#0000ff" # Blue
    colorDict[2] = "#000099" # Dark blue
    colorDict[3] = "#0099cc" # Light blue
    colorDict[4] = "#33cc33" # Green
    colorDict[5] = "#006600" # Dark Green
    colorDict[6] = "#99ff33" # Light green
    colorDict[7] = "#9900ff" # Purple
    colorDict[8] = "#6600cc" # Dark Purple
    colorDict[9] = "#9966ff" # Light purple
    colorDict[10] = "#cc00cc" # Pink
    colorDict[11] = "#990099" # Dark pink
    colorDict[12] = "#ff66ff" # Light pink
    colorDict[13] = "#ff0000" # Red
    colorDict[14] = "#800000" # Dark red
    colorDict[15] = "#ff5050" # Light red
    colorDict[16] = "#ffff00" # Yellow
    colorDict[17] = "#996600" # light brown
    colorDict[18] = "#ffff66" # Light Yellow
    colorDict[19] = "#666666" # Grey
    colorDict[20] = "#262626" # Dark grey
    colorDict[21] = "#cccccc" # Light Grey
    colorDict[22] = "#4d2600" # Brown
    colorDict[23] = "#ff9900" # Orange
    colorDict[24] = "#ff9933" # Light orange
    colorDict[25] = "#00e6e6" # Turquoise
    colorDict[26] = "#004d4d" # Dark turquoise
    colorDict[27] = "#333300" # Strange green
    colorDict[28] = "#cc3300" # Orange/red
    colorDict[29] = "#000000" # Black
    return colorDict


if __name__ == "__main__":
    main()