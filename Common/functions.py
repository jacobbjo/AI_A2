from Common.state import *
import matplotlib.pyplot as plt
import math
from moviepy.video.io.bindings import mplfig_to_npimage
import moviepy.editor as mpy

def createColorDict():
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
    """ Calculates distance between two points"""
    return np.linalg.norm(point2-point1)


def create_points(point_list):
    """ Creates Point objects and returns a list of them """
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
    """ Assigns points to the agents based on proximity to start and goal of the agent """
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
    """ Assigns points to the agents randomly"""
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
    """ Assigns points to the agents based on proximity to a line between start and goal of the agent """
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


def assign_points_line_plus(points, starts, goals, v_max):
    """
    Assigns points to the agents based on proximity to the current path the agent will travel. Starts by just a
    simple path between start and goal of the agent and throughout the iterations the path goes through other
    points of interest. Assignment is done by iterating over the points of interest to be assigned.
    """

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

    for i in range(len(routes)):
        route = routes[i]
        route_object = Route(route[0], route[-1], route[1:-1])
        state.add_route(route_object)

    return state


def assign_points_line_plus_plus(points, starts, goals, v_max):
    """
    Assigns points to the agents based on proximity to the current path the agent will travel. Starts by just a
    simple path between start and goal of the agent and throughout the iterations the path goes through other
    points of interest. Assignment is done by iterating over the AGENTS and assigning one point each. This results in
    an even distribution of POIs between the agents.
    """
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


def tabu_search(state):
    """ Test permutations of agent assignment to see if we can find a state that is better """

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
    """ Finds the best neighbor from the neighborhood """

    best_neighbor = good_neighborhood[0]

    for neighbor in good_neighborhood:
        if neighbor < best_neighbor:
            best_neighbor = neighbor

    return best_neighbor


def on_point(pos, points, limit):
    """ Determines whether a possition is within limit of a point in a list of points
    Returns after finding a single point even if there is more """
    for ind, point in enumerate(points):
        try:
            if np.linalg.norm(pos - point.xy) <limit:
                return ind
        except AttributeError:
            if np.linalg.norm(pos - point) < limit:
                return ind
    return -1


def on_point_rev(pos, points, limit, the_map):
    """ Determines whether a possition is within limit of a point in a list of points.
    Returns all points in the list that is within the limit"""

    index = []
    for ind, point in enumerate(points):
        try:
            if np.linalg.norm(pos - point.xy) < limit: #and the_map.clear_view(pos, point.xy):
                index.append(points[ind])
        except AttributeError:
            if np.linalg.norm(pos - point.xy) < limit: #and the_map.clear_view(pos, point.xy):
                index.append(points[ind])
    return index

def on_point_rev2(pos, points, limit, the_map):
    """ Determines whether a possition is within limit of a point in a list of points.
    Returns all points in the list that is within the limit"""

    index = []
    for ind, point in enumerate(points):
        try:
            if np.linalg.norm(pos - point.xy) < limit: #and the_map.clear_view(pos, point.xy):
                index.append(points[ind])
        except AttributeError:
            if np.linalg.norm(pos - point.xy) < limit: #and the_map.clear_view(pos, point.xy):
                index.append(points[ind])
    return index


def plot_agent_path(agents, starts, goals, points, v_max, dt, the_map):
    """ plots every step in the agents path  """

    colors = createColorDict()
    for i in range(len(agents[0].pos_hist)):
        print("Current pos: ", i)
        plt.clf()
        plt.axis("equal")
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
    """ Plots the map """
    colors = createColorDict()

    for i in range(len(starts)):
        color = colors[i+1]
        plt.plot(starts[i].xy[0], starts[i].xy[1], "o", c=color)
        try:
            plt.plot(goals[i].xy[0], goals[i].xy[1], "*", c=color)
        except IndexError:
            continue
    for i in range(len(points)):
        plt.plot(points[i].xy[0], points[i].xy[1], "x")
    the_map.plot_map()


def plot_agent_path_static(agents, starts, goals, points, the_map):
    """only plots the lines between the points"""

    colors = createColorDict()
    plot_map(starts, goals, points, the_map)
    for i, agent in enumerate(agents):
        color = colors[i+1]
        plt.plot([starts[i].xy[0], agent.pos_hist[0][0]],[starts[i].xy[1], agent.pos_hist[0][1]], c=color)
        for j in range(len(agent.pos_hist)-1):
            plt.plot([agent.pos_hist[j][0], agent.pos_hist[j+1][0]], [agent.pos_hist[j][1], agent.pos_hist[j+1][1]], c = color)
        plt.plot([goals[i].xy[0], agent.pos_hist[-1][0]], [goals[i].xy[1], agent.pos_hist[-1][1]], c=color)

#def check_if_pois_visited(agents, time_step, pois, limit):
#   """Returns a list with he pois that the agents visits at the current position"""
#   visited_pois = []
#   for agent in agents:
#       agent_pos = agent.pos_hist[time_step]
#       for poi in pois:
#           poi_xy = np.array([poi[0], poi[1]])
#           if np.linalg.norm(agent_pos - poi_xy) <= limit:
#               print(agent_pos)
#               print("Lägger till en punkt")
#               visited_pois.append(poi)
##       return visited_pois


def find_visited_points_dt(agents, pois, limit, the_map):
    """ Returns a list of when every poi is visited"""
    visited_points_dt = []
    the_pois = pois.copy()

    for time_step in range(len(agents[0].pos_hist)):
        visited_at_time_step = []
        for agent in agents:
            visited_points = on_point_rev(agent.pos_hist[time_step], the_pois, limit, the_map)
            for vis_point in visited_points:
                visited_at_time_step.append(vis_point)
            for vis_point in visited_points:
                the_pois.remove(vis_point)
        visited_points_dt.append(visited_at_time_step)

    return visited_points_dt


def find_visited_points_dt2(agents_path, pois, limit, the_map):
    """ Returns a list of when every poi is visited"""
    visited_points_dt = []
    the_pois = pois.copy()

    for time_step in range(len(agents_path[0][0])):
        visited_at_time_step = []
        for agent in agents_path:
            agent_point = np.array([agent[0][time_step], agent[1][time_step]])
            visited_points = on_point_rev2(agent_point, the_pois, limit, the_map)
            for vis_point in visited_points:
                visited_at_time_step.append(vis_point)
            for vis_point in visited_points:
                the_pois.remove(vis_point)
        visited_points_dt.append(visited_at_time_step)

    return visited_points_dt


def make_gif_poi(agent_paths, the_map, pois, poi_visited, title):
    """Makes a gif"""

    fig_mpl, ax = plt.subplots(1, figsize=(5, 5), facecolor='white')
    duration = len(agent_paths[0][0]) * the_map.vehicle_dt
    colors = createColorDict()

    ax.set_title(title)

    the_map.plot_map()

    for poi in pois:
        plt.plot(poi.xy[0], poi.xy[1], "x")

    plots = []

    # The paths of the agents should be np.arrays where each row correspond to a position at a given time

    for ind, agent_path in enumerate(agent_paths):
        # Plots start and goal
        plt.plot(agent_path[0][0], agent_path[1][0], "2", c=colors[ind + 1])
        plt.plot(agent_path[0][-1], agent_path[1][-1], "*", c=colors[ind + 1])

        point, = ax.plot(agent_path[0], agent_path[1], "o", c=colors[ind+1])
        plots.append(point)

    def make_frame_mpl(t):
        b = int(t * 10)
        try:
            for point in poi_visited[b]:
               plt.plot(point.xy[0], point.xy[1], "x", c="w")
        except IndexError:
            pass

        for ind, point in enumerate(plots):
            point.set_xdata(agent_paths[ind][0][b])
            point.set_ydata(agent_paths[ind][1][b])

        return mplfig_to_npimage(fig_mpl)  # RGB image of the figure

    animation = mpy.VideoClip(make_frame_mpl, duration=duration)
    animation.write_gif("test.gif", fps=10)


def write_to_file(filename, agents):
    """ Writes the path to a file """
    with open(filename, "w") as file:
        for agent in agents:
            points = ""
            for point in agent.pos_hist:
                points += str(point[0])+","+str(point[1])+" "
            file.write(points+"\n")


def read_from_file(filename):
    """ Reads the paths from a file """
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


def write_poi_to_file(filename, pois):
    """ write poi list to file"""

    with open(filename, "w") as file:
        for timestep in pois:
            points = ""
            for point in timestep:
                points += str(point.xy[0])+","+str(point.xy[1])+" "
            file.write(points+"\n")


def read_poi_from_file(filename):
    """ Reads poi list from file """
    pois = []

    with open(filename, "r") as file:
        for line in file:
            points = []
            for point in line.split(" "):
                try:
                    xy = np.array([float(point.split(",")[0]), float(point.split(",")[1])])

                    points.append(Point(xy, 0))

                except ValueError:
                    continue
            pois.append(points)
    return pois

def find_agent_route(agents, the_map):
    busy_agents = True
    dt = the_map.vehicle_dt
    v_max = the_map.vehicle_v_max
    neighbor_limit = 2

    while busy_agents:
        busy_agents = False
        new_vels = []

        for agent in agents:
            agent.save_pos()

            if agent.is_moving:
                print("Agent moving: ", agent.index)
                busy_agents = True
                # Get the new velocity for the moving agent
                new_vel = agent.find_best_vel(agents, neighbor_limit, the_map)
                new_vels.append(new_vel)

            else:
                # The agent is done and should stand still
                new_vels.append(np.zeros(2))

        for ind, agent in enumerate(agents):
            if agent.is_moving:
                agent.pos += new_vels[ind] * dt

                agent.vel = new_vels[ind]

                agent.check_route_status(v_max * dt)


def plot_trajectory(agents_paths, the_map):
    starts = create_points(the_map.start_positions)
    goals = create_points(the_map.goal_positions)
    plot_map(starts, goals, [], the_map)
    colors = createColorDict()
    for i, agent_path in enumerate(agents_paths):
        plt.plot(agent_path[0], agent_path[1], c= colors[i+1])

    plt.show()







