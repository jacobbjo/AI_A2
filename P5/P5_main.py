from P5.importJSON5 import Problem
from P5.leader import Leader
from P5.minion import Minion
from P5.functions_P5 import *
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

    dt_before_start = 0

    while not all_at_start(minions, the_map):
        leader.save_pos() # to keep correct history
        move_minions(the_map.leader_positions[0], the_map.leader_theta[0], minions, neighbor_limit, the_map)
        dt_before_start += 1

    for traj_pos, theta in zip(the_map.leader_positions[1:], the_map.leader_theta[1:]):
        leader.move_trajectory(traj_pos)
        move_minions(leader.pos, theta, minions, neighbor_limit, the_map)

    print(minions[0].pos_hist)

    filename = "P5.txt"
    write_to_file(filename, minions)

    agents_paths = read_from_file(filename)

    make_gif_poi(agents_paths, the_map, [], [], "Test P5")


if __name__ == "__main__":
    main()