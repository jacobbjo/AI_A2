from P5.importJSON5 import Problem
from P5.minion import Minion
from P5.functions_P5 import *
from Common.agent import Agent
from Common.functions import *

# Best:

def main():
    the_map = Problem("P25.json", "P25_26_traj.json")

    print(len(the_map.leader_positions))

    neighbor_limit = 3

    minions = []

    for ind, start_pos in enumerate(the_map.start_positions):
        minions.append(Minion(start_pos, the_map.formation_positions[ind+1],
                              the_map.formation_positions[0], the_map.leader_positions[-1]))

    print("hej")
    dt_before_start = 0

    while not all_at_start(minions, the_map):
        move_minions(the_map.formation_positions[0], minions, neighbor_limit, the_map)
        dt_before_start += 1
    print("hej2")
    print(minions[0].pos_hist)


    for leader_pos in the_map.leader_positions[1:]:
        move_minions(leader_pos, minions, neighbor_limit, the_map)
    print("hej3")

    while not all_at_goal(minions, the_map):
        move_minions(the_map.leader_positions[-1], minions, neighbor_limit, the_map)

    print("hej4")

    print(minions[0].pos_hist)

    plot_movement(leader_pos, minions, dt_before_start)











if __name__ == "__main__":
    main()