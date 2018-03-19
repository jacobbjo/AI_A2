import math
from P1.help_functions import *
from P1.importJSON1 import Problem
from P1.agent_P1 import Agent
import matplotlib.pyplot as plt

def plot_agents(agents):
    plt.clf()
    plt.axis("equal")
    plt.plot(5, 35, "o")
    plt.plot(35, 5, "o")
    plt.plot(5, 5, "o")
    plt.plot(35, 35, "o")
    for agent in agents:
        plt.plot(agent.pos[0], agent.pos[1], "o")
    plt.pause(0.005)

def plot_agent_path(agents, num_iterations):

    for i in range(num_iterations):
        plt.clf()
        plt.axis("equal")
        plt.plot(5, 35, "o")
        plt.plot(35, 5, "o")
        plt.plot(5, 5, "o")
        plt.plot(35, 35, "o")
        for agent in agents:
            try:
                pos = agent.pos_hist[i]
            except:
                pos = agent.pos_hist[-1]
            plt.plot(pos[0], pos[1], "o")
        plt.pause(0.05)




def collisions(agents):
    for i in range(len(agents)-1):
        j = i+1
        if i != j:
            if np.linalg.norm(agents[i].pos - agents[j].pos) < 1:
                print("KOLLISION MELLAN AGENT ", str(i)," OCH AGENT ", str(j))

left = math.atan2(-1, -3)
right = math.atan2(-2, -1)
vel = math.atan2(-1, -1)

the_map = Problem("source/P21.json")

#vmax = the_map.vehicle_v_max
vmax = 2
dt = the_map.vehicle_dt
radius = 0.5

neighbor_limit = vmax * dt * 10 + radius * 2

# -------- Creates the agents and stores them in list
agents = []
for i in range(len(the_map.start_positions)):
    agents.append(Agent(i, np.array(the_map.start_positions[i]), np.array(the_map.goal_positions[i]), radius))

# -------- Loops through the agents and finds their new position
agents_not_at_goal = True

# --------- plots the agents
#plot_agents(agents)

num_iterations = 0
while agents_not_at_goal:
    num_agents_at_goal = 0
    pos_differences = []
    new_vels = []
    #print(len(agents))
    for agent in agents:
        agent.pos_hist.append(np.copy(agent.pos))
        if round(np.linalg.norm(agent.goal - agent.pos), 2) < vmax * dt:
        #if abs(agent.pos[0] - agent.goal[0]) < vmax * dt and abs(agent.pos[1] - agent.goal[1]) < vmax * dt:
            # If an agents already is on goal we do not want to change its position
            num_agents_at_goal += 1
            pos_differences.append(np.zeros(2))
            new_vels.append(np.zeros(2))
            continue

        neighbors = get_neighbors(agent, agents, neighbor_limit)
        new_vel = agent.find_best_vel(neighbors, vmax)

        pos_change = new_vel * dt
        pos_differences.append(pos_change)
        new_vels.append(new_vel)

    for i in range(len(agents)):
        agents[i].pos += pos_differences[i]
        agents[i].vel = new_vels[i]
    #plot_agents(agents)
    if num_agents_at_goal == len(agents):
        agents_not_at_goal = False
    collisions(agents)
    num_iterations += 1
    print(num_agents_at_goal)

#for agent in agents:
#    print(agent.pos_hist)
#print(num_iterations)
#plot_agent_path(agents, num_iterations)
#plt.show()
print(num_iterations)

