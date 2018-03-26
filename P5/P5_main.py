from P5.importJSON5 import Problem
from P5.leader import Leader
from P5.minion import Minion
from P5.functions_P5 import *
from Common.functions import *

# Best:


def make_gif_traj(agent_paths, the_map,title):
    """Makes a gif"""

    fig_mpl, ax = plt.subplots(1, figsize=(10, 10), facecolor='white')

    duration = len(agent_paths[0][0]) * the_map.vehicle_dt
    colors = createColorDict()

    ax.set_title(title)

    the_map.plot_map()

    plots = []

    # The paths of the agents should be np.arrays where each row correspond to a position at a given time

    for ind, agent_path in enumerate(agent_paths):
        # Plots start and goal
        point, = ax.plot(agent_path[0], agent_path[1], "o", c=colors[ind+1])
        plots.append(point)

    def make_frame_mpl(t):
        b = int(t * 10)

        for ind, point in enumerate(plots):
            point.set_xdata(agent_paths[ind][0][b])
            point.set_ydata(agent_paths[ind][1][b])

        return mplfig_to_npimage(fig_mpl)  # RGB image of the figure

    animation = mpy.VideoClip(make_frame_mpl, duration=duration)
    animation.write_gif("test.gif", fps=10)

def main():
    the_map = Problem("P25.json", "P25_26_traj.json")

    # The paths are alreadt found, only construct the gifs
    agents_paths = read_from_file("P5.txt")

    make_gif_traj(agents_paths, the_map, "Test P5")

    #neighbor_limit = 3
    #leader = Leader(the_map.leader_positions[0], the_map.leader_positions[-1])
    #
    #minions = [leader]
    #
    #for ind, start_pos in enumerate(the_map.start_positions):
    #    minions.append(Minion(start_pos, the_map.formation_positions[ind+1],
    #                          the_map.formation_positions[0], the_map.leader_positions[-1]))
    #
    #dt_before_start = 0
    #
    #while not all_at_start(minions, the_map):
    #    leader.save_pos() # to keep correct history
    #    move_minions(the_map.leader_positions[0], the_map.leader_theta[0], minions, neighbor_limit, the_map)
    #    dt_before_start += 1
    #
    #for traj_pos, theta in zip(the_map.leader_positions[1:], the_map.leader_theta[1:]):
    #    leader.move_trajectory(traj_pos)
    #    move_minions(leader.pos, theta, minions, neighbor_limit, the_map)
    #
    #print(minions[0].pos_hist)
    #
    #filename = "P5.txt"
    #write_to_file(filename, minions)
    #
    #agents_paths = read_from_file(filename)
    #
    #make_gif_traj(agents_paths, the_map, "Test P5")


if __name__ == "__main__":
    main()