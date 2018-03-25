from Common.state import *
import matplotlib.pyplot as plt
import math


def createColorDictDist():
    # It is 29 districts
    colorDict = {}
    colorDict[1] = "#0000ff" # Blue
    colorDict[2] = "#000099" # Dark blue
    colorDict[3] = "#0099cc" # Light blue
    colorDict[4] = "#33cc33" # Green
    colorDict[5] = "#006600" # Dark Green
    colorDict[6] = "#99ff33" # Light green
    colorDict[7] = "#9900ff" # Purple
    colorDict[8] = "#6600cc" # Dark Purple
    colorDict[9] = "#9966ff" # Light purple
    colorDict[10] = "#cc00cc" # Pink
    colorDict[11] = "#990099" # Dark pink
    colorDict[12] = "#ff66ff" # Light pink
    colorDict[13] = "#ff0000" # Red
    colorDict[14] = "#800000" # Dark red
    colorDict[15] = "#ff5050" # Light red
    colorDict[16] = "#ffff00" # Yellow
    colorDict[17] = "#996600" # light brown
    colorDict[18] = "#ffff66" # Light Yellow
    colorDict[19] = "#666666" # Grey
    colorDict[20] = "#262626" # Dark grey
    colorDict[21] = "#cccccc" # Light Grey
    colorDict[22] = "#4d2600" # Brown
    colorDict[23] = "#ff9900" # Orange
    colorDict[24] = "#ff9933" # Light orange
    colorDict[25] = "#00e6e6" # Turquoise
    colorDict[26] = "#004d4d" # Dark turquoise
    colorDict[27] = "#333300" # Strange green
    colorDict[28] = "#cc3300" # Orange/red
    colorDict[29] = "#000000" # Black
    return colorDict


def calc_distance(point1, point2):
    return np.linalg.norm(point2-point1)


def create_points(point_list):
    points = []
    for i in range(len(point_list)):
        new_point = Point(point_list[i], i)
        points.append(new_point)
    return points


def find_line_eq(point1, point2):
    """Define a linear equation of the form ax + by + c = 0. Returns the a, b and c on the form (a, b, c)"""
    a = point1[1] - point2[1]
    b = point2[0] - point1[0]
    c = point1[0] * point2[1] - point2[0]*point1[1]

    return (a, b, c)


def dist_point_to_line(point, line_parameters):
    """Computes the distance from a point to a line. Returns the distance and the closest point on the line"""
    a = line_parameters[0]
    b = line_parameters[1]
    distance = abs(a*point[0] + b*point[1] + line_parameters[2])/math.sqrt(a**2 + b ** 2)
    close_x = (b * (b*point[0] - a*point[1]) - a*line_parameters[2])/(a**2 + b**2)
    close_y = (a * (-b*point[0] + a*point[1]) - b*line_parameters[2])/(a**2 + b**2)

    return distance, np.array([close_x, close_y])


def assign_points(points, starts, goals, v_max):
    routes = []
    for i in range(len(starts)):
        routes.append([])

    for i in range(len(points)):
        min_distance = float("infinity")
        min_index = -1

        for j in range(len(starts)):
            distance = min(calc_distance(points[i].xy, starts[j].xy), calc_distance(points[i].xy, goals[j].xy))
            if distance < min_distance:
                min_distance = distance
                min_index = j

        routes[min_index].append(points[i])
    state = State([], v_max)

    for i in range(len(routes)):
        route = routes[i]
        route_object = Route(starts[i], goals[i], route)
        state.add_route(route_object)

    return state


def assign_points_random(points,starts, goals, v_max):
    routes = []
    for i in range(len(starts)):
        routes.append([])

    for point in points:
        random_index = random.randint(0, len(starts)-1)
        routes[random_index].append(point)
    state = State([], v_max)

    for i in range(len(routes)):
        route = routes[i]
        route_object = Route(starts[i], goals[i], route)
        state.add_route(route_object)

    return state


def assign_points_line(points, starts, goals, v_max):
    routes = []

    for i in range(len(starts)):
        routes.append([])

    for i in range(len(points)):
        min_distance = float("infinity")
        min_index = -1

        for j in range(len(starts)):
            line = find_line_eq(starts[j].xy, goals[j].xy)
            distance, close_point = dist_point_to_line(points[i].xy, line)
            if not (starts[j].xy[1] < close_point[1] < goals[j].xy[1] or goals[j].xy[1] < close_point[1] < starts[j].xy[1]):
                distance = min(calc_distance(points[i].xy, starts[j].xy), calc_distance(points[i].xy, goals[j].xy))

            if distance < min_distance:
                min_distance = distance
                min_index = j

        routes[min_index].append(points[i])
    state = State([], v_max)

    for i in range(len(routes)):
        route = routes[i]
        route_object = Route(starts[i], goals[i], route)
        state.add_route(route_object)

    return state


def assign_points_line_plus_plus(points, starts, goals, v_max):
    routes = []
    points = points.copy()
    distance_limit = 3
    for i in range(len(starts)):
        routes.append([starts[i], goals[i]])
    counter = 0

    lonely_agents = []
    while len(lonely_agents) < len(starts):
        for agent_index in range(len(routes)):
            if agent_index in lonely_agents:
                continue
            agent_route = routes[agent_index]
            #print(agent_route)
            close_poi = None
            min_distance = float("infinity")
            point_index = 0
            for p_index in range(len(points)):
                for line_point_ind in range(len(agent_route)-1):
                    line = find_line_eq(agent_route[line_point_ind].xy, agent_route[line_point_ind + 1].xy)
                    distance, close_point = dist_point_to_line(points[p_index].xy, line)

                    if not (starts[agent_index].xy[1] < close_point[1] < goals[agent_index].xy[1] or goals[agent_index].xy[1] < close_point[1] < starts[agent_index].xy[1]):
                        distance = min(calc_distance(points[p_index].xy, starts[agent_index].xy), calc_distance(points[p_index].xy, goals[agent_index].xy))

                    if distance < min_distance:
                        min_distance = distance
                        close_poi = points[p_index]
                        point_index = line_point_ind
            if min_distance < distance_limit:
                routes[agent_index].insert(point_index+1, close_poi)
                points.remove(close_poi)
            else:
                lonely_agents.append(agent_index)
                print("appending agent: ", agent_index)

    print("points left: ", len(points))

    if len(points) > 0:
       for i in range(len(points)):
           min_distance = float("infinity")
           min_index = -1
           min_point_index = -1
           for j in range(len(starts)):
               point_list = routes[j]
               for p in range(len(point_list) - 1):
                   line = find_line_eq(point_list[p].xy, point_list[p + 1].xy)
                   distance, close_poi = dist_point_to_line(points[i].xy, line)
                   if not (starts[j].xy[1] < close_poi[1] < goals[j].xy[1] or goals[j].xy[1] < close_poi[1] <
                           starts[j].xy[1]):
                       distance = min(calc_distance(points[i].xy, starts[j].xy),
                                      calc_distance(points[i].xy, goals[j].xy))
                   counter += 1
                   if distance < min_distance:
                       min_distance = distance
                       min_index = j
                       min_point_index = p

           routes[min_index].insert(min_point_index + 1, points[i])

    state = State([], v_max)
    print(counter)
    for i in range(len(routes)):
        route = routes[i]
        route_object = Route(route[0], route[-1], route[1:-1])
        state.add_route(route_object)

    return state


def assign_points_line_plus(points, starts, goals, v_max):
    routes = []

    for i in range(len(starts)):
        routes.append([starts[i], goals[i]])
    counter = 0
    for i in range(len(points)):
        min_distance = float("infinity")
        min_index = -1
        min_point_index = -1
        for j in range(len(starts)):
            point_list = routes[j]
            for p in range(len(point_list)-1):
                line = find_line_eq(point_list[p].xy, point_list[p+1].xy)
                distance, close_point = dist_point_to_line(points[i].xy, line)
                if not (starts[j].xy[1] < close_point[1] < goals[j].xy[1] or goals[j].xy[1] < close_point[1] < starts[j].xy[1]):
                    distance = min(calc_distance(points[i].xy, starts[j].xy), calc_distance(points[i].xy, goals[j].xy))
                counter += 1
                if distance < min_distance:
                    min_distance = distance
                    min_index = j
                    min_point_index = p

        routes[min_index].insert(min_point_index+1, points[i])
    state = State([], v_max)
    print(counter)
    for i in range(len(routes)):
        route = routes[i]
        route_object = Route(route[0], route[-1], route[1:-1])
        state.add_route(route_object)

    return state


def tabu_search(state):
    num_rules = 10
    fail_limit = 20
    rule_changes = 0
    filter_out_rules = [None] * num_rules  # Contains tuples as (point, forbidden route index)
    best_state = state

    bad_neighborhoods = 0

    to_continue = True
    while to_continue:
        good_neighbors = []

        neighborhood = best_state.find_neighborhood()
        for neighbor in neighborhood:
            if filter_out_rules[0] is not None:
                for rule in filter_out_rules:
                    if rule is None:
                        break
                    if not rule[0] in neighbor.routes[rule[1]].route:
                        good_neighbors.append(neighbor)
            else:
                good_neighbors.append(neighbor)

        best_neighbor_state = best_neighbor(good_neighbors)
        if best_neighbor_state < best_state:
            best_state = best_neighbor_state

            if best_state.mutation is not None:
                filter_out_rules[rule_changes % num_rules] = best_state.mutation
                rule_changes += 1

            bad_neighborhoods = 0
        else:
            bad_neighborhoods +=1
            if bad_neighborhoods > 5:
                print(bad_neighborhoods)
            if bad_neighborhoods > fail_limit:
                to_continue = False

    return best_state


def best_neighbor(good_neighborhood):
    best_neighbor = good_neighborhood[0]

    for neighbor in good_neighborhood:
        if neighbor < best_neighbor:
            best_neighbor = neighbor

    return best_neighbor


def on_point(pos, points, v_max, dt):
    for ind, point in enumerate(points):
        if np.linalg.norm(pos - point.xy) < v_max*dt:
            return ind
    return -1


def plot_agent_path(agents, starts, goals, points, v_max, dt, the_map):
    colors = createColorDictDist()
    for i in range(len(agents[0].pos_hist)):
        print("Current pos: ", i)
        plt.clf()
        plt.axis("equal")
        #plt.plot(-5, 40, "o")
        #plt.plot(40, -5, "o")
        #plt.plot(-5, -5, "o")
        #plt.plot(40, 40, "o")
        plot_map(starts, goals, points, the_map)
        for ag_ind, agent in enumerate(agents):
            color = colors[ag_ind + 1]
            pos = agent.pos_hist[i]
            point_index = on_point(pos, points, v_max, dt)
            if point_index != -1:
                points.pop(point_index)
            plt.plot(pos[0], pos[1], "o", c = color)
        plt.pause(0.05)

    plt.show()


def plot_map(starts, goals, points, the_map):
    colors = createColorDictDist()

    for i in range(len(starts)):
        color = colors[i+1]
        plt.plot(starts[i].xy[0], starts[i].xy[1], "2", c=color)
        plt.plot(goals[i].xy[0], goals[i].xy[1], "*", c=color)
    for i in range(len(points)):
        plt.plot(points[i].xy[0], points[i].xy[1], "x")
    the_map.plot_map()


def plot_agent_path_static(agents, starts, goals, points, the_map):
    colors = createColorDictDist()
    plot_map(starts, goals, points, the_map)
    for i, agent in enumerate(agents):
        color = colors[i+1]
        plt.plot([starts[i].xy[0], agent.pos_hist[0][0]],[starts[i].xy[1], agent.pos_hist[0][1]], c=color)
        for j in range(len(agent.pos_hist)-1):
            plt.plot([agent.pos_hist[j][0], agent.pos_hist[j+1][0]], [agent.pos_hist[j][1], agent.pos_hist[j+1][1]], c = color)
        plt.plot([goals[i].xy[0], agent.pos_hist[-1][0]], [goals[i].xy[1], agent.pos_hist[-1][1]], c=color)


def write_to_file(filename, agents):
    with open(filename, "w") as file:
        for agent in agents:
            points = ""
            for point in agent.pos_hist:
                points += str(point[0])+","+str(point[1])+" "
            file.write(points+"\n")


def read_from_file(filename):
    agents_positions = []

    with open(filename, "r") as file:
        for line in file:
            positions = [[], []]
            for position in line.split(" "):
                try:
                    positions[0].append(float(position.split(",")[0]))  # x
                    positions[1].append(float(position.split(",")[1]))  # y

                except ValueError:
                    continue

            agents_positions.append(positions)

    return agents_positions
