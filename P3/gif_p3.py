from Common.functions import *
from P3.importJSON3 import Problem

filenameAgent = "P23X_best.txt"
#filenamePOI = "P23X_poi_best.txt"
the_map = Problem("P23X.json")

points = create_points(the_map.points_of_interest)

agents_paths = read_from_file(filenameAgent)
pois_dt = find_visited_points_dt2(agents_paths, create_points(the_map.points_of_interest), the_map.sensor_range, the_map)

write_poi_to_file("new_poi.txt", pois_dt)

print(len(agents_paths[0][0]))
poi_visited = read_poi_from_file("new_poi.txt")

make_gif_poi(agents_paths, the_map, points, poi_visited, "P23X")