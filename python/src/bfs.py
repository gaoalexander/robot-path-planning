import argparse
from collections import deque

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors


def visualize(grid):
    cmap = colors.ListedColormap(['white', 'pink', 'green'])
    plt.gca().set_aspect('equal')
    plt.pcolor(grid,cmap=cmap, edgecolors='k', linewidths=0.2)
    plt.show()

def is_valid(point, graph):
    y = point[0]
    x = point[1]

    height = len(graph)
    width = len(graph[0])

    if x >= 0 and x < width and y >= 0 and y < height:
        obstacle = graph[y, x]
        return not obstacle

    return False


def bfs(graph, start, end):
    d = [[-1, 0],
         [0, -1],
         [1, 0],
         [0, 1]]

    Q = deque()
    Q.append([start])

    visited = np.zeros((len(graph), len(graph[0])))
    visited[start[0], start[1]] = 1

    while Q:
        current_path = Q.pop()
        current_node = current_path[-1]

        if current_node[0] == end[0] and current_node[1] == end[1]:
            return current_path

        for i in range(len(d)):
            adj = np.array(current_node) + d[i]
            if is_valid(adj, graph) and not visited[adj[0], adj[1]]:
                new_path = list(current_path)
                new_path.append(adj)
                Q.appendleft(new_path)
                visited[adj[0], adj[1]] = 1

def apply_obstacles(map, obstacles):
    for obstacle in obstacles:
        for point in obstacle:
            map[point[1], point[0]] = 0.5

    return map


def main(args):

    W = 36
    H = 22

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

    obstacles = np.array([obstacle1, obstacle2])
    map = np.zeros((H, W))
    freespace = apply_obstacles(map, obstacles)
    start = [int(args.start[1]), int(args.start[0])]
    end = [int(args.end[1]), int(args.end[0])]

    path = bfs(freespace, start, end)

    if not path:
        print("No path found.")
        return 0

    print("PATH: ", path)

    #visualize results
    for node in path:
        freespace[node[0], node[1]] = 1

    visualize(freespace)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-s','--start', nargs='+', help='<Required> path start', required=True)
    parser.add_argument('-e','--end', nargs='+', help='<Required> path end point', required=True)
    args = parser.parse_args()

    main(args)
