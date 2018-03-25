from P2.importJSON2 import Problem
from Common.agent import Agent
from Common.functions import *

# Best 21.5

def main():
    the_map = Problem("P22.json")
    points_of_interest = the_map.points_of_interest
    points = create_points(the_map.points_of_interest)
    starts = create_points(the_map.start_positions)
    goals = create_points(the_map.goal_positions)
    dt = the_map.vehicle_dt
    v_max = the_map.vehicle_v_max
    invalid_point = np.array([21.47513602, 8.05249862])
    the_map.plot_map()
    plt.plot(invalid_point[0], invalid_point[1], "o")

    # Assign each point the the agent with the closest start or goalPoint
    init_state = assign_points_line(points, starts, goals, v_max)

    # Find the routes with tabu search
    final_state = tabu_search(init_state)

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
                #the_map.plot_map()
                #plt.plot(-5, 40, "o")
                #plt.plot(40, -5, "o")
                #plt.plot(-5, -5, "o")
                #plt.plot(40, 40, "o")
                #plt.plot(agent.pos[0], agent.pos[1], "o")
                #plt.pause(0.05)
                agent.vel = new_vels[ind]


                agent.check_route_status(v_max * dt)
    #plt.show()

    print("Plotting!")
    print("len(agents[0].pos_hist): ", len(agents[0].pos_hist))

    filename = "P2.txt"
    write_to_file(filename, agents)

    agents_paths = read_from_file(filename)

    visited_pois_dt = find_visited_points_dt(agents, points, the_map.vehicle_dt * the_map.vehicle_v_max)
    print(visited_pois_dt)

    make_gif_poi(agents_paths, the_map, points, visited_pois_dt, "Test P2")


    #plot_agent_path(agents,starts, goals, points, v_max, dt, the_map)
    #plot_agent_path_static(agents,starts, goals, points, the_map)
    #plt.show()




    # Show the point assignments
    #colors = createColorDictDist()
    #print(init_state.max_time)
    #print(final_state.max_time)

    # this is only for plot function
    init_routes = final_state.routes

   #for agent_index in range(len(init_routes)):
   #    color = colors[agent_index+1]
   #    for i in range(init_routes[agent_index].num_points):
#   #        plt.plot(init_routes[agent_index].route[i].xy[0], init_routes[agent_index].route[i].xy[1], "o", c=color)

   #    plt.plot(starts[agent_index].xy[0], starts[agent_index].xy[1], "x", c=color)
   #    plt.plot(goals[agent_index].xy[0], goals[agent_index].xy[1], "x", c=color)


   #plt.show()



if __name__ == "__main__":
    main()