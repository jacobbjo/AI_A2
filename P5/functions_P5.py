import matplotlib.pyplot as plt
from Common.functions import createColorDictDist

def all_at_goal(minions, the_map):
    for minion in minions:
        if not minion.at_goal(the_map):
            return False
    return True


def all_at_start(minions, the_map):
    for minion in minions[1:]:
        if not minion.at_start(the_map):
            return False
    return True

def move_minions(leader_pos, leader_theta, minions, limit, the_map):
    for minion in minions[1:]:
        minion.move(leader_pos, leader_theta, minions, limit, the_map)


def plot_movement(leader_pos, minions, dt_bf_start, the_map):
    colors = createColorDictDist()
    for i in range(len(minions[0].pos_hist)):
        plt.clf()
        plt.axis("equal")
        the_map.plot_map()
        plt.plot(the_map.formation_positions[0][0], the_map.formation_positions[0][1], "o", c=colors[1])
        for min, minion in enumerate(minions):
            plt.plot(minion.pos_hist[i][0], minion.pos_hist[i][1], "o", c=colors[min + 2])

        plt.pause(0.0005)

    plt.show()



