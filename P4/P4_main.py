from P4.importJSON4 import Problem
from Common.agent import Agent
from Common.functions import *

# Best: 25.2

def find_poi(all_poi, the_map):
    pois = []
    all_poi = all_poi.copy()

    while len(all_poi) > 0:
        print(len(all_poi))
        max_poi = []
        max_neigh = []
        i = 0
        for poi in all_poi:
            print("punkt: ", i)
            np_poi = np.array(poi)
            neighbors = []
            for other_poi in all_poi:  # Adds itself to neighbors as well
                np_other_poi = np.array(other_poi)

                if np.linalg.norm(np_poi - np_other_poi) <= the_map.sensor_range:
                    if the_map.clear_view(np_poi, np_other_poi):
                        neighbors.append(other_poi)
            if len(neighbors) > len(max_neigh):
                max_poi = poi
                max_neigh = neighbors
            i += 1
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
    the_map = Problem("P24.json")
    all_points = create_points(the_map.points_of_interest_np)
    points_of_interest = find_poi(the_map.points_of_interest, the_map)
    print(len(points_of_interest))

    points = create_points(points_of_interest)
    starts = create_points(the_map.start_positions)
    goals = create_points(the_map.goal_positions)
    dt = the_map.vehicle_dt
    v_max = the_map.vehicle_v_max

    # Assign each point the the agent with the closest start or goalPoint
    init_state = assign_points_line(points, starts, goals, v_max)
    print("Points assigned")

    # Find the routes with tabu search
    colors = createColorDict()

    numIteration = 20
    bestResutlt = len(read_from_file("P4_best.txt")[0][0])
    filenameAgents = "P4_best.txt"
    filenamePOI = "P4_poi_best.txt"

    for i in range(numIteration):

        print("Iteration ", i, " of ", numIteration)

        final_state = tabu_search(init_state)
        print("found Route")

        agents = []
        radius = 0.5
        neighbor_limit = 2  # vmax * dt * 10 + radius * 2

        for ind, route in enumerate(final_state.routes):
            agents.append(Agent(ind, route.start, route.goal, route.route, radius))

        find_agent_route(agents, the_map)
        visited_pois = find_visited_points_dt(agents, all_points, the_map.sensor_range, the_map)
        print(visited_pois)

        if len(agents[0].pos_hist) < bestResutlt:
            print("Found better: ", len(agents[0].pos_hist), ", before: ", bestResutlt)
            bestResutlt = len(agents[0].pos_hist)
            write_to_file(filenameAgents, agents)
            write_poi_to_file(filenamePOI, visited_pois)
        else:
            print(len(agents[0].pos_hist), " not better than ", bestResutlt)





    #print("Plotting!")
    #print("len(agents[0].pos_hist): ", len(agents[0].pos_hist))


    #filename = "P4.txt"
    #write_to_file(filename, agents)
    #agents_paths = read_from_file(filename)
    #visited_pois_dt = find_visited_points_dt(agents, all_points, the_map.sensor_range, the_map)
    #print(visited_pois_dt)

    #make_gif_poi(agents_paths, the_map, all_points, visited_pois_dt, "Test P4")


if __name__ == "__main__":
    main()