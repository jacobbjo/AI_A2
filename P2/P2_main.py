from P2.importJSON2 import Problem
from Common.agent import Agent
from Common.functions import *

# Best 21.5

def main():
    the_map = Problem("P22.json")
    #points_of_interest = the_map.points_of_interest
    points = create_points(the_map.points_of_interest)
    starts = create_points(the_map.start_positions)
    goals = create_points(the_map.goal_positions)
    dt = the_map.vehicle_dt
    v_max = the_map.vehicle_v_max

    # Assign each point the the agent with the closest start or goalPoint
    init_state = assign_points_line(points, starts, goals, v_max)

    numIteration = 20

    bestResutlt = len(read_from_file("P2_best.txt")[0][0])
    filenameAgents = "P2_best.txt"
    filenamePOI = "P2_poi_best.txt"

    for i in range(numIteration):
        print("Iteration ", i, " of ", numIteration)

        # Find the routes with tabu search
        final_state = tabu_search(init_state)

        # Create agents and find find their way to all points of interest while not hitting each other
        agents = []
        radius = 0.5
        neighbor_limit = 2  # vmax * dt * 10 + radius * 2

        for ind, route in enumerate(final_state.routes):
            agents.append(Agent(ind, route.start, route.goal, route.route, radius))

        find_agent_route(agents, the_map)
        visited_pois = find_visited_points_dt(agents, points, the_map.vehicle_dt * the_map.vehicle_v_max, the_map)

        if len(agents[0].pos_hist) < bestResutlt:
            print("Found better: ", len(agents[0].pos_hist), ", before: ", bestResutlt)
            bestResutlt = len(agents[0].pos_hist)
            write_to_file(filenameAgents, agents)
            write_poi_to_file(filenamePOI, visited_pois)
        else:
            print(len(agents[0].pos_hist), " not better than ", bestResutlt)

    # Plot the paths using pyplot
    #plot_agent_path(agents,starts, goals, points, v_max, dt, the_map)
    #plot_agent_path_static(agents,starts, goals, points, the_map)
    #plt.show()


if __name__ == "__main__":
    main()