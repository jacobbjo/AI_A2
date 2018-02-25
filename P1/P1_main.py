import numpy as np
import math
from help_functions import *
from importJSON1 import Problem
from agent import Agent
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

    plt.pause(0.05)


def collisions(agents):
    for i in range(len(agents)):
        for j in range(len(agents)):
            if i != j:
                if np.linalg.norm(agents[i].pos - agents[j].pos) < 1:
                    print("KOLLISION MELLAN AGENT ", str(i)," OCH AGENT ", str(j))

left = math.atan2(-1, -3)
right = math.atan2(-2, -1)
vel = math.atan2(-1, -1)

the_map = Problem("source/P21.json")

vmax = the_map.vehicle_v_max
dt = the_map.vehicle_dt
radius = 0.5
#print(vel_ang_ok(right, left, vel))

neighbor_limit = vmax * dt * 30

# -------- Creates the agents and stores them in list
agents = []
for i in range(len(the_map.start_positions) - 15):
    agents.append(Agent(i, np.array(the_map.start_positions[i]), np.array(the_map.goal_positions[i]), radius))

# -------- Loops through the agents and finds their new position
agents_not_at_goal = True

# --------- plots the agents
plot_agents(agents)

#start_fig = plt.figure()
#ax = start_fig.add_subplot(1,1,1)
#for agent in agents:
#    ax.plot(agent.pos[0], agent.pos[1], "o")


while agents_not_at_goal:
    num_agents_at_goal = 0
    pos_differences = []
    #print(len(agents))
    for agent in agents:
        if agent.pos[0] == agent.goal[0] and agent.pos[1] == agent.goal[1]: # If an agents already is on goal we do not want to change its position
            #print("inne i iffen")
            num_agents_at_goal += 1
            pos_differences.append(np.zeros(2))
            continue

        neighbors = get_neighbors(agent, agents, neighbor_limit)
        new_vel = agent.find_best_vel(neighbors, vmax)
        #print(new_vel)
        #if new_vel[0] == None:
        #    print("NONE")
        #    print(agent.pos)
        #    #agents.remove(agent)
        #    pos_differences.append(np.zeros(2))
        #    continue
        pos_change = new_vel * dt
        pos_differences.append(pos_change)

    for i in range(len(agents)):
        agents[i].pos += pos_differences[i]
    plot_agents(agents)
    if num_agents_at_goal == len(agents):
        agents_not_at_goal = False
    #print(num_agents_at_goal)
    collisions(agents)
plt.show()

