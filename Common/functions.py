import numpy as np

def assign_points_line(points, starts, goals, v_max):
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

def find_line_eq(point1, point2):
    """Define a linear equation of the form y = kx+m. Returns the k and m in the form (k, m)"""
    k = (point2[1] - point1[1])/(point2[0] - point1[0])
    m = point1[1] - k * point1[0]
    return (k, m)



p1 = np.array([1, 2])
p2 = np.array([2, 2.5])

print(find_line_eq(p1, p2))