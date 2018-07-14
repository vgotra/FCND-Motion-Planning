import numpy as np
import matplotlib.pyplot as plt

plt.rcParams['figure.figsize'] = 12, 12


def plot_map(grid, path, start_ne, goal_ne):
    plt.imshow(grid, cmap='Greys', origin='lower')
    plt.plot(start_ne[1], start_ne[0], 'rx')
    plt.plot(goal_ne[1], goal_ne[0], 'gx')
    plt.xlabel('EAST')
    plt.ylabel('NORTH')
    if path:
        pp = np.array(path)
        plt.plot(pp[:, 1], pp[:, 0], 'g')
        plt.scatter(pp[:, 1], pp[:, 0])
    plt.savefig('path_simple.png')


def plot_map_graph(grid, edges, path, start_ne, goal_ne):
    plt.imshow(grid, cmap='Greys', origin='lower')
    for e in edges:
        p1 = e[0]
        p2 = e[1]
        plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-')
    plt.plot(start_ne[1], start_ne[0], 'rx')
    plt.plot(goal_ne[1], goal_ne[0], 'gx')
    plt.xlabel('EAST')
    plt.ylabel('NORTH')
    if path:
        for i in range(len(path) - 1):
            p1 = path[i]
            p2 = path[i + 1]
            plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'g-')
    plt.savefig('path_graph.png')


def plot_map_skeleton(grid, skeleton, path, start_ne, goal_ne):
    plt.imshow(grid, cmap='Greys', origin='lower')
    plt.imshow(skeleton, cmap='Greys', origin='lower', alpha=0.7)
    plt.plot(start_ne[1], start_ne[0], 'rx')
    plt.plot(goal_ne[1], goal_ne[0], 'gx')
    plt.xlabel('EAST')
    plt.ylabel('NORTH')
    if path:
        pp = np.array(path)
        plt.plot(pp[:, 1], pp[:, 0], 'g')
        plt.scatter(pp[:, 1], pp[:, 0])
    plt.savefig('path_skeleton.png')