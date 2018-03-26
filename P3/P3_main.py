from P3.importJSON3 import Problem
from Common.agent import Agent
from Common.functions import *

# Best: 37.3 proximity
# Best: 36.3 line
# Best: 35.0 line plus

def find_poi(all_poi, sensor_r):
    poi = []
    all_poi = all_poi.copy()

    def remove_close_points(point, all_points, sensor_r):
        close_points = []
        np_point = np.array(point)
        for a_point in all_points:
            if a_point in close_points:
                continue
            np_apoint = np.array(a_point)
            if np.linalg.norm(np_point- np_apoint) <= sensor_r:
                close_points.append(a_point)
                #all_points.remove(a_point)
        for close_point in close_points:
            all_points.remove(close_point)

    while len(all_poi) > 0:
        point = all_poi[0]
        poi.append(point)
        remove_close_points(point, all_poi, sensor_r)

    return poi

def find_poi_better(all_poi, sensor_r):
    pois = []
    all_poi = all_poi.copy()

    while len(all_poi) > 0:
        print(len(all_poi))
        max_poi = []
        max_neigh = []

        for poi in all_poi:
            np_poi = np.array(poi)
            neighbors = []
            for other_poi in all_poi:  # Adds itself to neighbors as well
                np_other_poi = np.array(other_poi)
                if np.linalg.norm(np_poi - np_other_poi) <= sensor_r:
                    neighbors.append(other_poi)
            if len(neighbors) > len(max_neigh):
                max_poi = poi
                max_neigh = neighbors
        pois.append(max_poi)
        for point in max_neigh:
            all_poi.remove(point)
    return pois


def create_points(point_list):
    points = []
    for i in range(len(point_list)):
        new_point = Point(np.array(point_list[i]), i)
        points.append(new_point)
    return points


def main():
    the_map = Problem("P23.json")

    all_points = create_points(the_map.points_of_interest_np)
    points_of_interest = find_poi_better(the_map.points_of_interest, the_map.sensor_range)

    points = create_points(points_of_interest)
    starts = create_points(the_map.start_positions)
    goals = create_points(the_map.goal_positions)
    dt = the_map.vehicle_dt
    v_max = the_map.vehicle_v_max

    # Assign each point the the agent with the closest start or goalPoint
    init_state = assign_points_line_plus_plus(points, starts, goals, v_max)
    print("Points assigned")
    # Find the routes with tabu search
    colors = createColorDict()

    # Plots the point assignment
    init_routes = init_state.routes
    ax = plt.gca()
    for agent_index in range(len(init_routes)):
        color = colors[agent_index+1]
        for i in range(init_routes[agent_index].num_points):
            plt.plot(init_routes[agent_index].route[i].xy[0], init_routes[agent_index].route[i].xy[1], "o", c=color)
            circlen = plt.Circle((init_routes[agent_index].route[i].xy[0], init_routes[agent_index].route[i].xy[1]),
                                 the_map.sensor_range, color=color, fill=False)
            ax.add_artist(circlen)
            plt.plot(starts[agent_index].xy[0], starts[agent_index].xy[1], "2", c=color)
            plt.plot(goals[agent_index].xy[0], goals[agent_index].xy[1], "*", c=color)


    numIterations = 20
    bestResutlt = len(read_from_file("P3_best.txt")[0][0])
    filenameAgents = "P3_best.txt"
    filenamePOI = "P3_poi_best.txt"

    for i in range(numIterations):
        print("Iteration ", i, " of ", numIterations)

        final_state = tabu_search(init_state)
        print("found Route")

        busy_agents = True

        agents = []
        radius = 0.5
        neighbor_limit = 2  # vmax * dt * 10 + radius * 2

        for ind, route in enumerate(final_state.routes):
            agents.append(Agent(ind, route.start, route.goal, route.route, radius))

        find_agent_route(agents, the_map)
        visited_pois = find_visited_points_dt(agents, all_points, the_map.sensor_range, the_map)

        if len(agents[0].pos_hist) < bestResutlt:
            print("Found better: ", len(agents[0].pos_hist), ", before: ", bestResutlt)
            bestResutlt = len(agents[0].pos_hist)
            write_to_file(filenameAgents, agents)
            write_poi_to_file(filenamePOI, visited_pois)
        else:
            print(len(agents[0].pos_hist), " not better than ", bestResutlt)

    #print("Plotting!")
    #print("len(agents[0].pos_hist): ", len(agents[0].pos_hist))

    #filename = "P3.txt"
    #write_to_file(filename, agents)

    #agents_paths = read_from_file(filename)

    #visited_pois_dt = find_visited_points_dt(agents, all_points, the_map.sensor_range, the_map)
    #print("len visited: ", len(visited_pois_dt))
    #print(visited_pois_dt)

    #make_gif_poi(agents_paths, the_map, all_points, visited_pois_dt, "Test P3")


if __name__ == "__main__":
    main()
