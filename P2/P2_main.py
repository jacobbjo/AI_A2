from importJSON2 import Problem
import numpy as np
import matplotlib.pyplot as plt
from State import *
from agent_p2 import Agent



def calcDistance(point1, point2):
    return np.linalg.norm(point2-point1)


def assign_points(points, starts, goals, v_max):
    routes = []
    for i in range(len(starts)):
        routes.append([])

    for i in range(len(points)):
        min_distance = float("infinity")
        min_index = -1

        for j in range(len(starts)):
            distance = min(calcDistance(points[i].xy, starts[j].xy), calcDistance(points[i].xy, goals[j].xy))
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


def tabu_search(state):
    num_rules = 10
    fail_limit = 10
    rule_changes = 0
    filter_out_rules = [None] * num_rules  # Contains tuples as (point, forbidden route index)
    best_state = state

    bad_neighborhoods = 0

    to_continue = True
    while to_continue:
        good_neighbors = []

        neighborhood = best_state.find_neighborhood()
        for neighbor in neighborhood:
            if filter_out_rules[0] is not None:
                for rule in filter_out_rules:
                    if rule is None:
                        break
                    if not rule[0] in neighbor.routes[rule[1]].route:
                        good_neighbors.append(neighbor)
            else:
                good_neighbors.append(neighbor)

        best_neighbor_state = best_neighbor(good_neighbors)
        if best_neighbor_state < best_state:
            best_state = best_neighbor_state

            if best_state.mutation is not None:
                filter_out_rules[rule_changes % num_rules] = best_state.mutation
                rule_changes += 1

            bad_neighborhoods = 0
        else:
            bad_neighborhoods +=1
            if bad_neighborhoods > fail_limit:
                to_continue = False

    return best_state


def best_neighbor(good_neighborhood):
    best_neighbor = good_neighborhood[0]

    for neighbor in good_neighborhood:
        if neighbor < best_neighbor:
            best_neighbor = neighbor

    return best_neighbor


def main():
    the_map = Problem("P22.json")
    points = create_points(the_map.points_of_interest)
    starts = create_points(the_map.start_positions)
    goals = create_points(the_map.goal_positions)
    dt = the_map.vehicle_dt
    v_max = the_map.vehicle_v_max

    # Assign each point the the agent with the closest start or goalPoint
    init_state = assign_points(points, starts, goals, v_max)

    # Find the routes with tabu search
    final_state = tabu_search(init_state)

    busy_agents = True

    agents = []
    radius = 0.5
    neighbor_limit = 2  # vmax * dt * 10 + radius * 2
    vmax = the_map.vehicle_v_max

    for ind, route in enumerate(final_state.routes):
        agents.append(Agent(ind, route.start.xy, route.goal.xy, route.route, radius))




    while busy_agents:
        busy_agents = False
        new_vels = []

        for agent in agents:
            agent.save_pos()

            if agent.is_moving:
                busy_agents = True
                # Get the new velocity for the moving agent
                new_vel = agent.find_best_vel(agents, neighbor_limit, vmax)
                new_vels.append(new_vel)

            else:
                # The agent is done and should stand still
                new_vels.append(np.zeros(2))

        for ind, agent in enumerate(agents):
            agent.pos += new_vels[ind] * dt
            agent.vel = new_vels[ind]

            agent.check_route_status()










    # Show the point assignments
    colors = createColorDictDist()
    #print(init_state.max_time)
    #print(final_state.max_time)

    # this is only for plot function
    init_routes = final_state.routes

    for agent_index in range(len(init_routes)):
        color = colors[agent_index+1]
        for i in range(init_routes[agent_index].num_points):
            plt.plot(init_routes[agent_index].route[i].xy[0], init_routes[agent_index].route[i].xy[1], "o", c=color)

        plt.plot(starts[agent_index].xy[0], starts[agent_index].xy[1], "x", c=color)
        plt.plot(goals[agent_index].xy[0], goals[agent_index].xy[1], "x", c=color)


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