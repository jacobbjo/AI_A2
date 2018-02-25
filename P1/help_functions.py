import numpy as np

def get_neighbors(the_agent, agent_list, limit):
    """

    :param agent: The agent to whom the neighbors will be returned
    :param agent_list: list of all agents
    :param limit: The maximum distance which are considered neighborhood
    :return: list of all the neighbors to agent
    """

    neighbors = []

    for agent in agent_list:
        if agent == the_agent:
            continue
        if np.linalg.norm(agent.pos - the_agent.pos) < limit:
            neighbors.append(agent)

    return neighbors

def sum_range (ranges):
    out_ranges = [ranges[0]]

    for range in ranges:
        if(range[0]< 0):
            range[0] += (np.pi*2)

        if (range[1] < 0):
            range[1] += (np.pi * 2)

        for out_range in out_ranges:
            if out_range[0] <= range[0] <= out_range[1]:
                out_range[1] = range[1]
            else:
                out_ranges.append(range)

    return out_ranges