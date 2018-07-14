import numpy as np
import numpy.linalg as la
import matplotlib.pyplot as plt

plt.rcParams['figure.figsize'] = 12, 12


def prune_waypoints(original_path):
    """
    Can be improved to produce nice straight line for way
    """
    print('simplifying waypoints')
    if len(original_path) == 0:
        return []
    path = original_path[:]
    # get the last point
    goal_point = path.pop(len(path) - 1)

    result = []
    p1 = path.pop(0)
    result.append(p1)
    while len(path) > 0:
        if path:
            p2 = path.pop(0)
        else:
            break
        if path:
            p3 = path.pop(0)
        else:
            result.append(p2)
            break
        det = collinearity(p1, p2, p3)
        if not det:
            result.append(p2)
            result.append(p3)
        p1 = p3

    # very simple and stupid - but it enough to make waypoint without a lot of points
    pruned_result = []
    p1 = result[0]
    pruned_result.append(p1)
    for p2 in result[1:]:
        if not waypoints_are_close(p1, p2):
            pruned_result.append(p2)
            p1 = p2
    pruned_result.append(goal_point)

    return pruned_result


def collinearity(p1, p2, p3):
    det = p1[0] * (p2[1] - p3[1]) + p2[0] * (p3[1] - p1[1]) + p3[0] * (p1[1] - p2[1])
    return det == 0


def waypoints_are_close(p1, p2):
    diff = 4
    min_x = min(p1[0], p2[0])
    max_x = max(p1[0], p2[0])
    min_y = min(p1[1], p2[1])
    max_y = max(p1[1], p2[1])
    close = abs(max_x - min_x) <= diff and abs(max_y - min_y) <= diff
    return close


def are_close(col1, col2, max_diff):
    """This function used to compare values of collections with numeric data
    """
    if len(col1) != len(col2) or len(col1) != len(max_diff):
        raise ValueError("Different size of input collections")
    result = []
    for x, y, z in zip(col1, col2, max_diff):
        result.append(np.abs(np.abs(x) - np.abs(y)) < z)
    return np.array(result)


def closest_point(graph, current_point):
    """
    Compute the closest point in the `graph`
    to the `current_point`.
    """
    c_p = None
    dist = 100000
    for p in graph.nodes:
        d = la.norm(np.array(p) - np.array(current_point))
        if d < dist:
            c_p = p
            dist = d
    return c_p


def find_start_goal(skeleton, start, goal):
    skeleton_cells = np.transpose(skeleton.nonzero())
    start_min_dist = np.linalg.norm(np.array(start) - np.array(skeleton_cells), axis=1).argmin()
    near_start = skeleton_cells[start_min_dist]
    goal_min_dist = np.linalg.norm(np.array(goal) - np.array(skeleton_cells), axis=1).argmin()
    near_goal = skeleton_cells[goal_min_dist]
    return near_start, near_goal
