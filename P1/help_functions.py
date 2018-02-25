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

