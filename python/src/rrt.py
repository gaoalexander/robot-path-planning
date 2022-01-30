import argparse
import math
import random
from enum import Enum
from collections import deque

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors


MAX_DISTANCE = 2


def visualize(freespace, tree_init, tree_goal, path, response):
    vertices_init = np.array([[v[0], v[1]] for v, _ in tree_init.tree.items()])
    vertices_goal = np.array([[v[0], v[1]] for v, _ in tree_goal.tree.items()])

    edges_init = []
    for v, adj in tree_init.tree.items():
        for each in adj:
            edges_init.append([[v[1], each[1]], [v[0], each[0]]])

    edges_goal = []
    for v, adj in tree_goal.tree.items():
        for each in adj:
            edges_goal.append([[v[1], each[1]], [v[0], each[0]]])

    for e in edges_init:
        plt.plot(e[0], e[1], color='red')
    for e in edges_goal:
        plt.plot(e[0], e[1], color='blue')

    if response == rrt_response.SUCCESS:
        for i in range(len(path) - 1):
            plt.plot([path[i][1], path[i + 1][1]], [path[i][0], path[i + 1][0]], color='green')

    cmap = colors.ListedColormap(['white', 'pink', 'green'])
    plt.gca().set_aspect('equal')
    plt.pcolor(freespace, cmap=cmap, edgecolors='k', linewidths=0.2)

    plt.scatter(vertices_init[:, 1], vertices_init[:, 0], color='red')
    plt.scatter(vertices_goal[:, 1], vertices_goal[:, 0], color='blue')

    plt.show()


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class Tree:
    def __init__(self, node):
        self.tree = {node: set()}

    def add_vertex(self, v):
        self.tree[v] = set()

    def add_edge(self, from_node, to_node):
        self.tree[from_node].add(to_node)
        self.tree[to_node].add(from_node)


class extend_response(Enum):
    TRAPPED = 0
    ADVANCED = 1
    REACHED = 2


class rrt_response(Enum):
    FAILURE = 0
    SUCCESS = 1


def apply_obstacles(map, obstacles):
    for obstacle in obstacles:
        for point in obstacle:
            map[point[1], point[0]] = 0.5
    return map


def random_config(freespace):
    H = len(freespace)
    W = len(freespace[0])

    return np.array([random.uniform(0, H - 1), random.uniform(0, W - 1)])


def euclidean_distance(p1, p2):
    return math.sqrt((p2[1] - p1[1])**2 + (p2[0] - p1[0])**2)


def nearest_neighbor(tree, q):
    nearest = None
    min_distance = math.inf

    for v, _ in tree.tree.items():
        d = euclidean_distance(np.array([v[0], v[1]]), q)
        if d < min_distance:
            min_distance = d
            nearest = v

    return nearest


def get_path(tree, start, end):
    d = [[-1, 0],
         [0, -1],
         [1, 0],
         [0, 1]]

    Q = deque()
    Q.append([start])

    visited = set()
    visited.add((start[0], start[1]))

    while Q:
        current_path = Q.pop()
        current_node = current_path[-1]

        if current_node[0] == end[0] and current_node[1] == end[1]:
            return current_path

        for adj in tree.tree[tuple(current_node)]:
            if adj not in visited:
                new_path = list(current_path)
                new_path.append(adj)
                Q.appendleft(new_path)
                visited.add(tuple([adj[0], adj[1]]))


def connected(freespace, p1, p2):
    target_direction = p2 - p1
    current = p1.copy()

    i = 1
    while i <= 100 and freespace[math.floor(current[0]), math.floor(current[1])] == 0 and euclidean_distance(current, p2) < MAX_DISTANCE:
        current += 0.01 * target_direction
        i += 1

    if abs(current[0] - p2[0]) < 0.01 and abs(current[1] - p2[1]) < 0.01:
        return True
    else:
        return False


def extend(tree, q, freespace):
    q_near = nearest_neighbor(tree, q)
    q_new = q_near
    target_direction = q - q_near

    i = 1
    while i <= 100 and freespace[math.floor(q_new[0]), math.floor(q_new[1])] == 0 and euclidean_distance(q_near, q_new) < MAX_DISTANCE:
        q_new += 0.01 * target_direction
        i += 1

    if q_near[0] == q_new[0] and q_near[1] == q_new[1]:
        return extend_response.TRAPPED, None
    else:
        tree.add_vertex(tuple(q_new))
        tree.add_edge(tuple(q_near), tuple(q_new))
        return extend_response.ADVANCED, q_new


def rrt_connect(start, end, freespace, K):
    tree_init = Tree(start)
    tree_goal = Tree(end)

    count = 0
    for i in range(K):
        count += 1
        q_rand = random_config(freespace)
        response, q_new = extend(tree_init, q_rand, freespace)

        if response != extend_response.TRAPPED:
            tree_goal_nearest = nearest_neighbor(tree_goal, q_new)
            if connected(freespace, q_new, tree_goal_nearest):
                if start in tree_init.tree:
                    path = get_path(tree_init, start, q_new) + get_path(tree_goal, tree_goal_nearest, end)
                else:
                    path = get_path(tree_goal, start, tree_goal_nearest) + get_path(tree_init, q_new, end)
                return rrt_response.SUCCESS, path, tree_init, tree_goal, count

        tree_init, tree_goal = tree_goal, tree_init
    return rrt_response.FAILURE, None, tree_init, tree_goal, count


def main(args):

    W = 30
    H = 17

    obstacle1 = [[16, 5], [16, 6], [16, 7], [16, 8], [16, 9], [16, 10], [16, 11], [16, 12],
                 [17, 6], [17, 7], [17, 8], [17, 9], [17, 10], [17, 11], [17, 12], [17, 13],
                 [18, 7], [18, 8], [18, 9], [18, 10], [18, 11], [18, 12], [18, 13], [18, 14],
                 [19, 8], [19, 9], [19, 10], [19, 11], [19, 12], [19, 13], [19, 14], [19, 15], [19, 16],
                 [20, 8], [20, 9], [20, 10], [20, 11], [20, 12], [20, 13], [20, 14], [20, 15], [20, 16], [20, 17],
                 [21, 8], [21, 9], [21, 10], [21, 11], [21, 12], [21, 13], [21, 14], [21, 15], [21, 16], [21, 17], [21, 18],
                 [22, 8], [22, 9], [22, 10], [22, 11], [22, 12], [22, 13], [22, 14], [22, 15], [22, 16], [22, 17], [22, 18],
                 [23, 7], [23, 8], [23, 9], [23, 10], [23, 11], [23, 12], [23, 13], [23, 14], [23, 15], [23, 16], [23, 17],
                 [24, 7], [24, 8], [24, 9], [24, 10], [24, 12], [24, 13], [24, 14], [24, 15], [24, 16],
                 [25, 7], [25, 8], [25, 9], [25, 12], [25, 13], [25, 14], [25, 15],
                 [26, 6], [26, 7], [26, 8], [26, 12], [26, 13], [26, 14],
                 [27, 6], [27, 7], [27, 8],
                 [28, 6], [28, 7],
                 [29, 6]]

    obstacle2 = [[5, 8], [5, 9],
                 [6, 8], [6, 9], [6, 10], [6, 11], [6, 12],
                 [7, 10], [7, 11], [7, 12], [7, 13], [7, 13],
                 [8, 13], [8, 12], [8, 11], [8, 10],
                 [9, 12], [9, 11]]

    for i in range(len(obstacle1)):
        obstacle1[i] = [obstacle1[i][0]- 2, obstacle1[i][1] - 2]

    for i in range(len(obstacle2)):
        obstacle2[i] = [obstacle2[i][0] - 1, obstacle2[i][1] - 5]

    obstacles = np.array([obstacle1, obstacle2])
    map = np.zeros((H, W))
    freespace = apply_obstacles(map, obstacles)

    start = (int(args.start[1]), int(args.start[0]))
    end = (int(args.end[1]), int(args.end[0]))

    K = 2000

    response, path, tree_init, tree_goal, count = rrt_connect(start, end, freespace, K)

    visualize(freespace, tree_init, tree_goal, path, response)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-s','--start', nargs='+', help='<Required> path start', required=True)
    parser.add_argument('-e','--end', nargs='+', help='<Required> path end point', required=True)
    args = parser.parse_args()

    main(args)
