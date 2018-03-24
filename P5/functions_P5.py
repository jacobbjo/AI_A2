import matplotlib.pyplot as plt
from Common.functions import createColorDictDist

def all_at_goal(minions, the_map):
    for minion in minions:
        if not minion.at_goal(the_map):
            return False
    return True


def all_at_start(minions, the_map):
    for minion in minions:
        if not minion.at_start(the_map):
            return False
    return True

def move_minions(leader_pos, minions, limit, the_map):
    for minion in minions:
        minion.move(leader_pos, minions, limit, the_map)


def plot_movement(leader_pos, minions, dt_bf_start):
    colors = createColorDictDist()
    for i in range(len(minions[0].pos_hist)):
        plt.clf()
        plt.axis("equal")
        if i < dt_bf_start:
            for min, minion in enumerate(minions):
                plt.plot(minion.pos_hist[0], minion.pos_hist[1], "o", c=colors[min+2])
        plt.pause(0.05)

        for j, minion in enumerate(minions):
            plt.plot()

    plt.show()



