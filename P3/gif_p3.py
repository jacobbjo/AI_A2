from Common.functions import *
from P3.importJSON3 import Problem

filenameAgent = "P23X_best.txt"
filenamePOI = "P23X_poi_best.txt"
the_map = Problem("P23X.json")

points = create_points(the_map.points_of_interest)

agents_paths = read_from_file(filenameAgent)
print(len(agents_paths[0][0]))
poi_visited = read_poi_from_file(filenamePOI)

make_gif_poi(agents_paths, the_map, points, poi_visited, "P23X")