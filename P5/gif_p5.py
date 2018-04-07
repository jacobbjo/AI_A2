from P5.P5_main import make_gif_traj
from Common.functions import read_from_file
from P5.importJSON5 import Problem

the_map = Problem("P25X.json", "P25_26_traj.json")
filename = "P25X.txt"
agents_paths = read_from_file(filename)

make_gif_traj(agents_paths, the_map, "P25X")