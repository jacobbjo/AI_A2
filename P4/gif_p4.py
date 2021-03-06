from Common.functions import *
from P4.importJSON4 import Problem

filenameAgent = "P24X_best.txt"
filenamePOI = "P24X_poi_best.txt"
the_map = Problem("P24X.json")

points = create_points(the_map.points_of_interest)

agents_paths = read_from_file(filenameAgent)
print(len(agents_paths[0][0]))
poi_visited = read_poi_from_file(filenamePOI)

make_gif_poi(agents_paths, the_map, points, poi_visited, "P24X")