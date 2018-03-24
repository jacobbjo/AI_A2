from P5.importJSON5 import Problem
from P5.leader import Leader
from P5.minion import Minion
from P5.functions_P5 import *
from Common.agent import Agent
from Common.functions import *

# Best:

def main():
    the_map = Problem("P25.json", "P25_26_traj.json")

    print(len(the_map.leader_positions))

    neighbor_limit = 3
    leader = Leader(the_map.leader_positions[0], the_map.leader_positions[-1])

    minions = [leader]

    for ind, start_pos in enumerate(the_map.start_positions):
        minions.append(Minion(start_pos, the_map.formation_positions[ind+1],
                              the_map.formation_positions[0], the_map.leader_positions[-1]))

    print("hej")
    dt_before_start = 0

    while not all_at_start(minions, the_map):
        leader.save_pos() # to keep correct history
        move_minions(the_map.leader_positions[0], the_map.leader_theta[0], minions, neighbor_limit, the_map)
        dt_before_start += 1
    print("hej2")
    #print(minions[0].pos_hist)

    #while not leader.at_start(the_map):
    #    leader.move(the_map)
    #    move_minions(leader.pos, np.pi/2, minions, neighbor_limit, the_map)
    print("hej2.5")
    for traj_pos, theta in zip(the_map.leader_positions[1:], the_map.leader_theta[1:]):
        leader.move_trajectory(traj_pos)
        move_minions(leader.pos, theta, minions, neighbor_limit, the_map)
    print("hej3")

    #while not all_at_goal(minions, the_map):
    #    move_minions(the_map.leader_positions[-1], the_map.leader_theta[-1], minions, neighbor_limit, the_map)

    print("hej4")

    print(minions[0].pos_hist)

    plot_movement(the_map.leader_positions, minions, dt_before_start, the_map)











if __name__ == "__main__":
    main()