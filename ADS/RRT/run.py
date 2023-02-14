import matplotlib.pyplot as plt
import random
import math
from rrt import RRT, Node

# Define the start and goal positions
start = (0, 0)
goal = (5, 5)

# Define the area in which the random points can be generated
rand_area = (-2, 7, -2, 7)

# Define the obstacles in the environment
obstacle_list = [(2, 2, 1), (3, 3, 1), (1, 3, 1)]

# Define the RRT parameters
rrt = RRT(start, goal, obstacle_list, rand_area)

# Plan the path using RRT
path = rrt.planning()

# Plot the path and obstacles
fig, ax = plt.subplots()
ax.set(xlim=rand_area[0:2], ylim=rand_area[2:], aspect=1)
for obstacle in obstacle_list:
    circle = plt.Circle((obstacle[0], obstacle[1]), obstacle[2], color='k')
    ax.add_patch(circle)
for i in range(len(path) - 1):
    ax.plot([path[i][0], path[i + 1][0]], [path[i][1], path[i + 1][1]], 'g')
    plt.scatter(path[i][0], path[i][1])
ax.plot(start[0], start[1], 'bs')
ax.plot(goal[0], goal[1], 'rs')
ax.axis("equal")
ax.grid(True)
plt.show()
