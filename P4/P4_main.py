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
    #print(len(the_map.points_of_interest))
    #print(points_of_interest)
    #
    # Plot for visualizing the points of interest
    #the_map.plot_map()
    #for point in points_of_interest:
    #    plt.plot(point[0], point[1], "*")
    #for point in the_map.points_of_interest:
    #    plt.plot(point[0], point[1], "x")
    #plt.show()

    points = create_points(points_of_interest)
    starts = create_points(the_map.start_positions)
    goals = create_points(the_map.goal_positions)
    dt = the_map.vehicle_dt
    v_max = the_map.vehicle_v_max
    #sensor_range = the_map.sensor_range

    # Assign each point the the agent with the closest start or goalPoint
    init_state = assign_points_line(points, starts, goals, v_max)
    print("Points assigned")
    # Find the routes with tabu search
    colors = createColorDictDist()
    #for i in range(len(init_state.routes)):



    final_state = tabu_search(init_state)
    print("found Route")

    busy_agents = True

    agents = []
    radius = 0.5
    neighbor_limit = 2  # vmax * dt * 10 + radius * 2

    for ind, route in enumerate(final_state.routes):
        agents.append(Agent(ind, route.start, route.goal, route.route, radius))

    while busy_agents:
        busy_agents = False
        new_vels = []

        for agent in agents:
            agent.save_pos()

            if agent.is_moving:
                busy_agents = True
                # Get the new velocity for the moving agent
                new_vel = agent.find_best_vel(agents, neighbor_limit, the_map)
                new_vels.append(new_vel)

            else:
                # The agent is done and should stand still
                new_vels.append(np.zeros(2))

        for ind, agent in enumerate(agents):
            if agent.is_moving:

                agent.pos += new_vels[ind] * dt
                agent.vel = new_vels[ind]

                agent.check_route_status(v_max * dt)

    print("Plotting!")
    print("len(agents[0].pos_hist): ", len(agents[0].pos_hist))


    #plot_agent_path(agents,starts, goals, points, v_max, dt, the_map)

    filename = "P4.txt"
    write_to_file(filename, agents)

    agents_paths = read_from_file(filename)

    visited_pois_dt = find_visited_points_dt(agents, all_points, the_map.sensor_range, the_map)
    print(visited_pois_dt)

    make_gif_poi(agents_paths, the_map, all_points, visited_pois_dt, "Test P4")



    # Show the point assignments
    #colors = createColorDictDist()
    #print(init_state.max_time)
    #print(final_state.max_time)

    # this is only for plot function
    #init_routes = final_state.routes

   #for agent_index in range(len(init_routes)):
   #    color = colors[agent_index+1]
   #    for i in range(init_routes[agent_index].num_points):
#   #        plt.plot(init_routes[agent_index].route[i].xy[0], init_routes[agent_index].route[i].xy[1], "o", c=color)

   #    plt.plot(starts[agent_index].xy[0], starts[agent_index].xy[1], "x", c=color)
   #    plt.plot(goals[agent_index].xy[0], goals[agent_index].xy[1], "x", c=color)


   #plt.show()


if __name__ == "__main__":
    main()